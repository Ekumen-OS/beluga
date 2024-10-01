// Copyright 2022-2023 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BELUGA_SENSOR_LIKELIHOOD_FIELD_MODEL_HPP
#define BELUGA_SENSOR_LIKELIHOOD_FIELD_MODEL_HPP

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <beluga/actions/overlay.hpp>
#include <beluga/algorithm/distance_map.hpp>
#include <beluga/sensor/data/occupancy_grid.hpp>
#include <beluga/sensor/data/value_grid.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

/**
 * \file
 * \brief Implementation of a likelihood field sensor model for range finders.
 */

namespace beluga {

/// Parameters used to construct a LikelihoodFieldModel instance.
/**
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 6.4, particularly Table 6.3.
 */
struct LikelihoodFieldModelParam {
  /// Maximum distance to obstacle.
  /**
   * When creating a distance map, if the distance to an obstacle is higher than the value specified here,
   * then this value will be used.
   */
  double max_obstacle_distance = 100.0;
  /// Maximum range of a laser ray.
  double max_laser_distance = 2.0;
  /// Weight used to combine the probability of hitting an obstacle.
  double z_hit = 0.5;
  /// Weight used to combine the probability of random noise in perception.
  double z_random = 0.5;
  /// Standard deviation of a gaussian centered arounds obstacles.
  /**
   * Used to calculate the probability of the obstacle being hit.
   */
  double sigma_hit = 0.2;
  /// Whether to model unknown space or assume it free.
  bool model_unknown_space = false;
};

/// Likelihood field sensor model for range finders.
/**
 * This model relies on a pre-computed likelihood map of the environment.
 * It is less computationally intensive than the beluga::BeamSensorModel
 * because no ray-tracing is required, and it can also provide better
 * performance in environments with non-smooth occupation maps. See
 * Probabilistic Robotics \cite thrun2005probabilistic, Chapter 6.4,
 * for further reference.
 *
 * \note This class satisfies \ref SensorModelPage.
 *
 * \tparam OccupancyGrid Type representing an occupancy grid.
 *  It must satisfy \ref OccupancyGrid2Page.
 */
template <class OccupancyGrid>
class LikelihoodFieldModel {
 public:
  /// State type of a particle.
  using state_type = Sophus::SE2d;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: a point cloud for the range finder.
  using measurement_type = std::vector<std::pair<double, double>>;
  /// Map representation type.
  using map_type = OccupancyGrid;
  /// Parameter type that the constructor uses to configure the likelihood field model.
  using param_type = LikelihoodFieldModelParam;

  /// Constructs a LikelihoodFieldModel instance.
  /**
   * \param params Parameters to configure this instance.
   *  See beluga::LikelihoodFieldModelParam for details.
   * \param grid Occupancy grid representing the static map that the sensor model
   *  uses to compute a likelihood field for lidar hits and compute importance weights
   *  for particle states.
   */
  explicit LikelihoodFieldModel(const param_type& params, const map_type& grid)
      : params_{params},
        likelihood_field_{make_likelihood_field(params, grid)},
        world_to_likelihood_field_transform_{grid.origin().inverse()} {}

  /// Returns the likelihood field, constructed from the provided map.
  [[nodiscard]] const auto& likelihood_field() const { return likelihood_field_; }

  /// Returns a state weighting function conditioned on 2D lidar hits.
  /**
   * \param points 2D lidar hit points in the reference frame of particle states.
   * \return a state weighting function satisfying \ref StateWeightingFunctionPage
   *  and borrowing a reference to this sensor model (and thus their lifetime are bound).
   */
  [[nodiscard]] auto operator()(measurement_type&& points) const {
    return [this, points = std::move(points)](const state_type& state) -> weight_type {
      const auto transform = world_to_likelihood_field_transform_ * state;
      const auto x_offset = transform.translation().x();
      const auto y_offset = transform.translation().y();
      const auto cos_theta = transform.so2().unit_complex().x();
      const auto sin_theta = transform.so2().unit_complex().y();
      const auto unknown_space_occupancy_prob = static_cast<float>(1. / params_.max_laser_distance);
      return std::transform_reduce(
          points.cbegin(), points.cend(), 1.0, std::plus{},
          [this, x_offset, y_offset, cos_theta, sin_theta, unknown_space_occupancy_prob](const auto& point) {
            // Transform the end point of the laser to the grid local coordinate system.
            // Not using Eigen/Sophus because they make the routine x10 slower.
            // See `benchmark_likelihood_field_model.cpp` for reference.
            const auto x = point.first * cos_theta - point.second * sin_theta + x_offset;
            const auto y = point.first * sin_theta + point.second * cos_theta + y_offset;
            const auto pz =
                static_cast<double>(likelihood_field_.data_near(x, y).value_or(unknown_space_occupancy_prob));
            // TODO(glpuga): Investigate why AMCL and QuickMCL both use this formula for the weight.
            // See https://github.com/Ekumen-OS/beluga/issues/153
            return pz * pz * pz;
          });
    };
  }

  /// Update the sensor model with a new occupancy grid map.
  /**
   * This method re-computes the underlying likelihood field.
   *
   * \param grid New occupancy grid representing the static map.
   */
  void update_map(const map_type& grid) {
    likelihood_field_ = make_likelihood_field(params_, grid);
    world_to_likelihood_field_transform_ = grid.origin().inverse();
  }

 private:
  param_type params_;
  ValueGrid2<float> likelihood_field_;
  Sophus::SE2d world_to_likelihood_field_transform_;

  static ValueGrid2<float> make_likelihood_field(const LikelihoodFieldModelParam& params, const OccupancyGrid& grid) {
    const auto squared_distance = [&grid](std::size_t first, std::size_t second) {
      return static_cast<float>((grid.coordinates_at(first) - grid.coordinates_at(second)).squaredNorm());
    };

    const auto squared_max_distance = static_cast<float>(params.max_obstacle_distance * params.max_obstacle_distance);
    const auto truncate_to_max_distance = [squared_max_distance](auto squared_distance) {
      return std::min(squared_distance, squared_max_distance);
    };

    /// Pre-computed variables
    const double two_squared_sigma = 2 * params.sigma_hit * params.sigma_hit;
    assert(two_squared_sigma > 0.0);

    const double amplitude = params.z_hit / (params.sigma_hit * std::sqrt(2 * Sophus::Constants<double>::pi()));
    assert(amplitude > 0.0);

    const double offset = params.z_random / params.max_laser_distance;

    const auto to_likelihood = [=](double squared_distance) {
      return amplitude * std::exp(-squared_distance / two_squared_sigma) + offset;
    };

    const auto neighborhood = [&grid](std::size_t index) { return grid.neighborhood4(index); };

    // determine distances to obstacles and calculate likelihood values in-place
    // to minimize memory usage when dealing with large maps
    auto distance_map = nearest_obstacle_distance_map(grid.obstacle_mask(), squared_distance, neighborhood);

    if (params.model_unknown_space) {
      const double inverse_max_distance = 1 / params.max_laser_distance;
      const double background_distance = -two_squared_sigma * std::log((inverse_max_distance - offset) / amplitude);

      distance_map |= beluga::actions::overlay(grid.unknown_mask(), background_distance);
    }

    std::transform(distance_map.begin(), distance_map.end(), distance_map.begin(), truncate_to_max_distance);
    std::transform(distance_map.begin(), distance_map.end(), distance_map.begin(), to_likelihood);

    return ValueGrid2<float>{std::move(distance_map), grid.width(), grid.resolution()};
  }
};

}  // namespace beluga

#endif

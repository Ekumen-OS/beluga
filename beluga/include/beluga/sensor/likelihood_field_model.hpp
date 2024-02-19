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
  double max_obstacle_distance;
  /// Maximum range of a laser ray.
  double max_laser_distance;
  /// Weight used to combine the probability of hitting an obstacle.
  double z_hit;
  /// Weight used to combine the probability of random noise in perception.
  double z_random;
  /// Standard deviation of a gaussian centered arounds obstacles.
  /**
   * Used to calculate the probability of the obstacle being hit.
   */
  double sigma_hit;
};

/// Likelihood field sensor model for range finders.
/**
 * This class satisfies \ref SensorModelPage.
 *
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 6.4.
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
      const auto unknown_space_occupancy_prob = 1. / params_.max_laser_distance;
      // TODO(glpuga): Investigate why AMCL and QuickMCL both use this formula for the weight.
      // See https://github.com/Ekumen-OS/beluga/issues/153
      const auto unknown_space_occupancy_likelihood_cubed =
          unknown_space_occupancy_prob * unknown_space_occupancy_prob * unknown_space_occupancy_prob;
      return std::transform_reduce(
          points.cbegin(), points.cend(), 1.0, std::plus{},
          [this, x_offset, y_offset, cos_theta, sin_theta,
           unknown_space_occupancy_likelihood_cubed](const auto& point) {
            // Transform the end point of the laser to the grid local coordinate system.
            // Not using Eigen/Sophus because they make the routine x10 slower.
            // See `benchmark_likelihood_field_model.cpp` for reference.
            const auto x = point.first * cos_theta - point.second * sin_theta + x_offset;
            const auto y = point.first * sin_theta + point.second * cos_theta + y_offset;
            // for performance, we store the likelihood already elevated to the cube
            return likelihood_field_.data_near(x, y).value_or(unknown_space_occupancy_likelihood_cubed);
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
  ValueGrid2<double> likelihood_field_;
  Sophus::SE2d world_to_likelihood_field_transform_;

  static ValueGrid2<double> make_likelihood_field(const LikelihoodFieldModelParam& params, const OccupancyGrid& grid) {
    const auto squared_distance = [&grid,
                                   squared_max_distance = params.max_obstacle_distance * params.max_obstacle_distance](
                                      std::size_t first, std::size_t second) {
      return std::min((grid.coordinates_at(first) - grid.coordinates_at(second)).squaredNorm(), squared_max_distance);
    };

    const auto neighborhood = [&grid](std::size_t index) { return grid.neighborhood4(index); };

    const auto distance_map = nearest_obstacle_distance_map(grid.obstacle_data(), squared_distance, neighborhood);

    const auto to_likelihood = [amplitude =
                                    params.z_hit / (params.sigma_hit * std::sqrt(2 * Sophus::Constants<double>::pi())),
                                two_squared_sigma = 2 * params.sigma_hit * params.sigma_hit,
                                offset = params.z_random / params.max_laser_distance](double squared_distance) {
      return amplitude * std::exp(-squared_distance / two_squared_sigma) + offset;
    };

    // we store the likelihood elevated to the cube to save a few runtime computations
    // when calculating the importance weight
    const auto to_the_cube = [](auto likelihood) { return likelihood * likelihood * likelihood; };

    auto likelihood_data = distance_map | ranges::views::transform(to_likelihood) |
                           ranges::views::transform(to_the_cube) | ranges::to<std::vector>;
    return ValueGrid2<double>{std::move(likelihood_data), grid.width(), grid.resolution()};
  }
};

}  // namespace beluga

#endif

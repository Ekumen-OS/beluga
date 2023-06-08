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
 * This class implements the LaserSensorModelInterface2d interface
 * and satisfies \ref SensorModelPage.
 *
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 6.4.
 *
 * \tparam Mixin The mixed-in type with no particular requirements.
 * \tparam OccupancyGrid Type representing an occupancy grid.
 *  It must satisfy \ref OccupancyGrid2Page.
 */
template <class Mixin, class OccupancyGrid>
class LikelihoodFieldModel : public Mixin {
 public:
  /// State type of a particle.
  using state_type = Sophus::SE2d;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: a point cloud for the range finder.
  using measurement_type = std::vector<std::pair<double, double>>;

  /// Parameter type that the constructor uses to configure the likelihood field model.
  using param_type = LikelihoodFieldModelParam;

  /// Constructs a LikelihoodFieldModel instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param params Parameters to configure this instance.
   *  See beluga::LikelihoodFieldModelParam for details.
   * \param grid Occupancy grid representing the static map.
   * \param ...rest Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit LikelihoodFieldModel(const param_type& params, OccupancyGrid grid, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...),
        params_{params},
        grid_{std::move(grid)},
        free_states_{compute_free_states(grid_)},
        likelihood_field_{make_likelihood_field(params, grid_)} {}

  /// Returns the likelihood field, constructed from the provided map.
  [[nodiscard]] const auto& likelihood_field() const { return likelihood_field_; }

  // TODO(ivanpauno): is sensor model the best place for this?
  // Maybe the map could be provided by a different part of the mixin,
  // and that part could be used to generate the random state.
  /// Generates a random particle state.
  /**
   * The generated state is an unoccupied cell of the grid, any free cell is sampled uniformly.
   * The rotation is as well sampled uniformly.
   *
   * \tparam Generator  A random number generator that must satisfy the
   *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
   *  requirements.
   * \param gen An uniform random bit generator object.
   * \return The generated random state.
   */
  template <class Generator>
  [[nodiscard]] state_type make_random_state(Generator& gen) const {
    auto index_distribution = std::uniform_int_distribution<std::size_t>{0, free_states_.size() - 1};
    return Sophus::SE2d{Sophus::SO2d::sampleUniform(gen), free_states_[index_distribution(gen)]};
  }

  /// Gets the importance weight for a particle with the provided state.
  /**
   * \param state State of the particle to calculate its importance weight.
   * \return Calculated importance weight.
   */
  [[nodiscard]] weight_type importance_weight(const state_type& state) const {
    const auto transform = grid_.origin().inverse() * state;
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
        points_.cbegin(), points_.cend(), 1.0, std::plus{},
        [this, x_offset, y_offset, cos_theta, sin_theta, unknown_space_occupancy_likelihood_cubed](const auto& point) {
          // Transform the end point of the laser to the grid local coordinate system.
          // Not using Eigen/Sophus because they make the routine x10 slower.
          // See `benchmark_likelihood_field_model.cpp` for reference.
          const auto x = point.first * cos_theta - point.second * sin_theta + x_offset;
          const auto y = point.first * sin_theta + point.second * cos_theta + y_offset;
          // for performance, we store the likelihood already elevated to the cube
          return likelihood_field_.data_near(x, y).value_or(unknown_space_occupancy_likelihood_cubed);
        });
  }

  /// \copydoc LaserSensorModelInterface2d::update_sensor(measurement_type&& points)
  void update_sensor(measurement_type&& points) final { points_ = std::move(points); }

  /// \copydoc LaserSensorModelInterface2d::update_map(Map&& map)
  void update_map(OccupancyGrid&& map) final {
    grid_ = std::move(map);
    free_states_ = compute_free_states(grid_);
    likelihood_field_ = make_likelihood_field(params_, grid_);
  }

 private:
  param_type params_;
  OccupancyGrid grid_;
  std::vector<Eigen::Vector2d> free_states_;
  ValueGrid2<double> likelihood_field_;
  std::vector<std::pair<double, double>> points_;

  static std::vector<Eigen::Vector2d> compute_free_states(const OccupancyGrid& grid) {
    constexpr auto kFrame = OccupancyGrid::Frame::kGlobal;
    return grid.coordinates_for(grid.free_cells(), kFrame) | ranges::to<std::vector>;
  }

  static ValueGrid2<double> make_likelihood_field(const LikelihoodFieldModelParam& params, const OccupancyGrid& grid) {
    const auto squared_max_distance = params.max_obstacle_distance * params.max_obstacle_distance;

    const auto squared_distance = [&grid, squared_max_distance](const auto& first, const auto& second) {
      return std::min((grid.coordinates_at(first) - grid.coordinates_at(second)).squaredNorm(), squared_max_distance);
    };

    const auto neighborhood = [&grid](const auto& cell) { return grid.neighborhood4(cell); };

    const auto distance_map = nearest_obstacle_distance_map(
        grid.width(), grid.height(), grid.obstacle_data(), squared_distance, neighborhood);

    const auto amplitude = params.z_hit / (params.sigma_hit * std::sqrt(2 * Sophus::Constants<double>::pi()));
    const auto two_squared_sigma = 2 * params.sigma_hit * params.sigma_hit;
    const auto offset = params.z_random / params.max_laser_distance;

    const auto to_likelihood = [amplitude, two_squared_sigma, offset](double squared_distance) {
      return amplitude * std::exp(-squared_distance / two_squared_sigma) + offset;
    };

    // we store the likelihood elevated to the cube to save a few runtime computations
    // when calculating the importance weight
    const auto to_the_cube = [](auto likelihood) { return likelihood * likelihood * likelihood; };

    auto likelihood_data = distance_map | ranges::views::transform(to_likelihood) |
                           ranges::views::transform(to_the_cube) | ranges::to<std::vector>;
    return ValueGrid2<double>(std::move(likelihood_data), grid.width(), grid.resolution());
  }
};

}  // namespace beluga

#endif

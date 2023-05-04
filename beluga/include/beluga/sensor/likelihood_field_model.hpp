// Copyright 2022 Ekumen, Inc.
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
#include <shared_mutex>
#include <vector>

#include <beluga/algorithm/distance_map.hpp>
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

/**
 * \page OccupancyGrid2dPage Beluga named requirements: OccupancyGrid2d
 *
 * A type `G` satisfies the `OccupancyGrid2d` requirements if.
 * Given `g` a possible const instance of `G`:
 * - `g.size()` returns a `std::size_t`, representing the occupancy grid size.
 * - `g.data()` returns a const [rancom access range](https://en.cppreference.com/w/cpp/ranges/random_access_range),
 *   with value type `C`.
 * - `g.neighbors()` returns a range with value type std::size_t.
 *   The elements of the range are valid cell indexes, i.e. for each `i` of the returned range `i < g.size()` is true
 *   and `g.data()[i]` is valid.
 * - `g.origin()` return the occupancy grid origin as a `Sophus::SE2d`.
 * - Given possibly const values `x` and `y` of type `int`, `g.valid(x, y)` returns whether a cell at those
 *   discrete coordinates exists.
 * - Given possibly const `Eigen::Vector2i` `c`, `g.valid(c)` is equivalent to `g.point(c.x(), c.y())`.
 * - Given a possibly const `std::size_t` `i` less than `g.size()`, `g.point(i)` returns the continuous
 *   coordinates of the centroid of the cell at index `i` in the grid frame as a `Eigen::Vector2d`.
 * - Given possibly const values `x` and `y` of type `int`, `g.point(x, y)` returns the continuous
 *   coordinates in the grid frame of the centroid of the cell at those discrete coordinates
 *   as a `Eigen::Vector2d`.
 * - Given a possibly const `Eigen::Vector2i` `c`, `g.point(c)` is equivalent to `g.point(c.x(), c.y())`.
 * - Given possibly const values `x` and `y` of type `double`, `g.index(x, y)` returns a `std::size`,
 *   representing the index of the cell.
 * - Given a possibly const `Eigen::Vector2d` `p`, `g.index(p)` is equivalent to `g.index(p.x(), p.y())`.
 * - Given possibly const values `x` and `y` of type `int`, `g.index(x, y)` returns a `std::size`,
 *   representing the index of the cell at those discrete coordinates.
 * - Given a possibly const `Eigen::Vector2i` `c`, `g.index(c)` is equivalent to `g.index(c.x(), c.y())`.
 *
 * Given c a possible const instance of C:
 * - OccupancyGrid::Traits::is_free(c) returns true if the cell is free, false if not.
 * - OccupancyGrid::Traits::is_occupied(c) returns true if the cell is occupied, false if not.
 * - OccupancyGrid::Traits::is_unknown(c) returns true if it is unknown whether the cell is occupied or not,
 *   else false.
 */

/// Likelihood field sensor model for range finders.
/**
 * This class implements the LaserSensorModelInterface2d interface
 * and satisfies \ref SensorModelPage.
 *
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 6.4.
 *
 * \tparam Mixin The mixed-in type with no particular requirements.
 * \tparam OccupancyGrid Type representing an occupancy grid.
 *  It must satisfy \ref OccupancyGrid2dPage.
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
  explicit LikelihoodFieldModel(const param_type& params, const OccupancyGrid& grid, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...),
        params_{params},
        grid_{grid},
        free_cells_{make_free_cells_vector(grid)},
        likelihood_field_{make_likelihood_field(params, grid)} {}

  /// Returns the likelihood field, constructed from the provided map.
  const auto& likelihood_field() const { return likelihood_field_; }

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
    auto index_distribution = std::uniform_int_distribution<std::size_t>{0, free_cells_.size() - 1};
    return Sophus::SE2d{
        Sophus::SO2d::sampleUniform(gen), grid_.origin() * grid_.point(free_cells_[index_distribution(gen)])};
  }

  /// Gets the importance weight for a particle with the provided state.
  /**
   * \param state State of the particle to calculate its importance weight.
   * \return Calculated importance weight.
   */
  [[nodiscard]] weight_type importance_weight(const state_type& state) const {
    const auto transform = grid_.origin().inverse() * state;
    const auto lock = std::shared_lock<std::shared_mutex>{points_mutex_};
    // TODO(glpuga): Investigate why AMCL and QuickMCL both use this formula for the weight.
    // See https://github.com/ekumenlabs/beluga/issues/153
    return std::transform_reduce(
        points_.cbegin(), points_.cend(), 1.0, std::plus{},
        [this, x_offset = transform.translation().x(), y_offset = transform.translation().y(),
         cos_theta = transform.so2().unit_complex().x(), sin_theta = transform.so2().unit_complex().y(),
         unknown_space_occupancy_prob = 1. / params_.max_laser_distance](const auto& point) {
          // Transform the end point of the laser to the global coordinate system.
          // Not using Eigen/Sophus because they make the routine x10 slower.
          // See `benchmark_likelihood_field_model.cpp` for reference.
          const auto x = point.first * cos_theta - point.second * sin_theta + x_offset;
          const auto y = point.first * sin_theta + point.second * cos_theta + y_offset;
          const auto index = grid_.index(x, y);
          const auto pz = index < likelihood_field_.size() ? likelihood_field_[index] : unknown_space_occupancy_prob;
          return pz * pz * pz;
        });
  }

  /// \copydoc LaserSensorModelInterface2d::update_sensor(measurement_type&& points)
  void update_sensor(measurement_type&& points) final {
    const auto lock = std::lock_guard<std::shared_mutex>{points_mutex_};
    points_ = std::move(points);
  }

 private:
  param_type params_;
  OccupancyGrid grid_;
  std::vector<std::size_t> free_cells_;
  std::vector<double> likelihood_field_;
  std::vector<std::pair<double, double>> points_;
  mutable std::shared_mutex points_mutex_;

  static std::vector<std::size_t> make_free_cells_vector(const OccupancyGrid& grid) {
    auto free_cells = std::vector<std::size_t>{};
    free_cells.reserve(grid.size());
    for (std::size_t index = 0; index < grid.size(); ++index) {
      if (OccupancyGrid::Traits::is_free(grid.data()[index])) {
        free_cells.push_back(index);
      }
    }
    return free_cells;
  }

  static std::vector<double> make_likelihood_field(const LikelihoodFieldModelParam& params, const OccupancyGrid& grid) {
    const auto obstacle_map = grid.data() | ranges::views::transform(&OccupancyGrid::Traits::is_occupied);

    auto squared_distance = [&grid, squared_max_distance = params.max_obstacle_distance * params.max_obstacle_distance](
                                std::size_t first, std::size_t second) {
      return std::min((grid.point(first) - grid.point(second)).squaredNorm(), squared_max_distance);
    };

    const auto distance_map = nearest_obstacle_distance_map(
        obstacle_map, squared_distance, [&grid](std::size_t index) { return grid.neighbors(index); });

    auto to_likelihood = [amplitude =
                              params.z_hit / (params.sigma_hit * std::sqrt(2 * Sophus::Constants<double>::pi())),
                          two_squared_sigma = 2 * params.sigma_hit * params.sigma_hit,
                          offset = params.z_random / params.max_laser_distance](double squared_distance) {
      return amplitude * std::exp(-squared_distance / two_squared_sigma) + offset;
    };

    return distance_map | ranges::views::transform(to_likelihood) | ranges::to<std::vector>;
  }
};

}  // namespace beluga

#endif

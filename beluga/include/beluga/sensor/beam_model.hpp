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

#ifndef BELUGA_SENSOR_BEAM_MODEL_HPP
#define BELUGA_SENSOR_BEAM_MODEL_HPP

#include <algorithm>
#include <beluga/algorithm/raycasting.hpp>
#include <cmath>
#include <random>
#include <shared_mutex>
#include <vector>

#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

/**
 * \file
 * \brief Implementation of a beam sensor model for range finders.
 */

namespace beluga {

/// Parameters used to construct a BeamSensorModel instance.
/**
 * See Probabilistic Robotics \cite thrun2005probabilistic table 6.2.
 */
struct BeamModelParams {
  /// Weight associated with good but noisy readings.
  double z_hit{0.5};
  /// Weight associated with unexpected obstacles.
  double z_short{0.5};
  /// Weight associated with max range readings.
  double z_max{0.05};
  /// Weight associated with random readings.
  double z_rand{0.05};
  /// Standard deviation of the gaussian noise associated with hits.
  double sigma_hit{0.2};
  /// Intrinsic parameter assoaciated with short readings distribution.
  double lambda_short{0.1};
  /// Maximum range of a laser ray. This is the value expected in case of a miss.
  double laser_max_range{60};
  /// How many evenly-spaced beams in each scan to be used when applying the sensor model.
  std::size_t max_beams{60};
};

/// Beam sensor model for range finders.
/**
 * This class implements the LaserSensorModelInterface2d interface
 * and satisfies \ref SensorModelPage.
 *
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 6.2.
 *
 * \tparam Mixin The mixed-in type with no particular requirements.
 * \tparam OccupancyGrid Type representing an occupancy grid.
 *  It must satisfy \ref OccupancyGrid2dPage.
 */
template <class Mixin, class OccupancyGrid>
class BeamSensorModel : public Mixin {
 public:
  /// State type of a particle.
  using state_type = Sophus::SE2d;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: a point cloud for the range finder.
  using measurement_type = std::vector<std::pair<double, double>>;

  /// Parameter type that the constructor uses to configure the beam sensor model.
  using param_type = BeamModelParams;

  /// Constructs a BeamSensorModel instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param params Parameters to configure this instance.
   *  See beluga::BeamModelParams for details.
   * \param grid Occupancy grid representing the static map.
   * \param ...rest Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit BeamSensorModel(const param_type& params, const OccupancyGrid& grid, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), grid_{grid}, free_cells_{make_free_cells_vector(grid)}, options_{params} {}

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
    const auto lock = std::shared_lock<std::shared_mutex>{points_mutex_};
    return std::transform_reduce(points_.cbegin(), points_.cend(), 0.0, std::plus{}, [this, state](const auto& point) {
      // TODO(Ramiro): We're converting from range + bearing to cartesian points in the ROS node, but we want range +
      // bearing here. We might want to make that conversion in the likelihood model instead, and let the measurement
      // type be range, bearing instead of x, y.

      // Compute the range according to the measurement.
      const double observation_range = std::sqrt(point.first * point.first + point.second * point.second);
      const auto beam_bearing = Sophus::SO2d{point.first, point.second};

      // Compute the range according to the map (raycasting).
      const double map_range = raycast(grid_, state, beam_bearing, options_.laser_max_range);

      double pz = 0.0;

      // 1: Correct range with local measurement noise.
      const double z = observation_range - map_range;
      pz += options_.z_hit * std::exp(-(z * z) / (2. * options_.sigma_hit * options_.sigma_hit));

      // 2: Unexpected objects.
      if (z < 0) {
        pz += options_.z_short * options_.lambda_short * std::exp(-options_.lambda_short * observation_range);
      }

      // 3 and 4: Max range return or random return.
      if (observation_range >= options_.laser_max_range) {
        pz += options_.z_max;
      } else {
        pz += options_.z_rand / options_.laser_max_range;
      }

      // Use the same ad-hoc accumulating strategy that nav2 uses.
      return pz * pz * pz;
    });
  }

  /// \copydoc LaserSensorModelInterface2d::update_sensor(measurement_type&& points)
  void update_sensor(measurement_type&& points) final {
    const auto lock = std::lock_guard<std::shared_mutex>{points_mutex_};
    points_ = std::move(points);
  }

 private:
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

  OccupancyGrid grid_;
  std::vector<std::pair<double, double>> points_;
  std::vector<std::size_t> free_cells_;
  mutable std::shared_mutex points_mutex_;
  param_type options_;
};

}  // namespace beluga

#endif

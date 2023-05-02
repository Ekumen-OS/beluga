// Copyright 2023 Ekumen, Inc.
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
#include <cmath>
#include <random>
#include <shared_mutex>
#include <vector>

#include <beluga/algorithm/raycasting.hpp>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>

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
struct BeamModelParam {
  /// Weight associated with good but noisy readings.
  double z_hit{0.5};
  /// Weight associated with unexpected obstacles.
  double z_short{0.05};
  /// Weight associated with max range readings.
  double z_max{0.05};
  /// Weight associated with random readings.
  double z_rand{0.4};
  /// Standard deviation of the gaussian noise associated with hits.
  double sigma_hit{0.2};
  /// Intrinsic parameter assoaciated with short readings distribution.
  double lambda_short{0.1};
  /// Maximum range of a laser ray. This is the value expected in case of a miss.
  double beam_max_range{100.};
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
 * \tparam Scan Type representing a planar scan.
 *  It must satisfy \ref Scan2dPage.
 */
template <class Mixin, class OccupancyGrid, class Scan>
class BeamSensorModel : public Mixin {
 public:
  /// State type of a particle.
  using state_type = Sophus::SE2d;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: a point cloud for the range finder.
  using measurement_type = Scan;

  /// Parameter type that the constructor uses to configure the beam sensor model.
  using param_type = BeamModelParam;

  /// Constructs a BeamSensorModel instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param params Parameters to configure this instance.
   *   See beluga::BeamModelParam for details.
   * \param grid Occupancy grid representing the static map.
   * \param ...rest Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit BeamSensorModel(const param_type& params, const OccupancyGrid& grid, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), grid_{grid}, free_cells_{make_free_cells_vector(grid)}, params_{params} {}

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
    const auto lock = std::shared_lock<std::shared_mutex>{scan_mutex_};
    const auto beam = ray2d{grid_, state * scan_pose_, params_.beam_max_range};
    return std::transform_reduce(
        scan_points_.begin(), scan_points_.end(), 0., std::plus{}, [this, &beam](const auto& point) {
          // Fetch range according to latest scan.
          const double z = point.x();

          // Compute range according to the map (raycasting).
          const double z_mean = beam.cast(Sophus::SO2d{point.y()}).value_or(params_.beam_max_range);

          // 1: Correct range with local measurement noise.
          const double eta_hit =
              2. / (std::erf((params_.beam_max_range - z_mean) / (std::sqrt(2.) * params_.sigma_hit)) -
                    std::erf(-z_mean / (std::sqrt(2.) * params_.sigma_hit)));
          const double d = (z - z_mean) / params_.sigma_hit;
          const double n = 1. / (std::sqrt(2. * M_PI) * params_.sigma_hit);
          double pz = params_.z_hit * eta_hit * n * std::exp(-(d * d) / 2.);

          // 2: Unexpected objects.
          if (z < z_mean) {
            const double eta_short = 1. / (1. - std::exp(-params_.lambda_short * z_mean));
            pz += params_.z_short * params_.lambda_short * eta_short * std::exp(-params_.lambda_short * z);
          }

          // 3 and 4: Max range return or random return.
          if (z < params_.beam_max_range) {
            pz += params_.z_rand / params_.beam_max_range;
          } else {
            pz += params_.z_max;
          }

          // TODO(glpuga): Investigate why AMCL and QuickMCL both use this formula for the weight.
          // See https://github.com/ekumenlabs/beluga/issues/153
          return pz * pz * pz;
        });
  }

  /// \copydoc LaserSensorModelInterface2d::update_sensor(measurement_type&& scan)
  void update_sensor(Scan&& scan) final {
    const auto lock = std::lock_guard<std::shared_mutex>{scan_mutex_};
    scan_points_ = scan.points_in_polar_coordinates() | ranges::views::filter([](const auto& point) {
                     // r cannot be negative, by definition
                     return point.x() >= 0.;
                   }) |
                   ranges::to<std::vector>;
    scan_pose_ = scan.origin();
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
  Sophus::SE2d scan_pose_;
  std::vector<Eigen::Vector2d> scan_points_;
  std::vector<std::size_t> free_cells_;
  mutable std::shared_mutex scan_mutex_;
  param_type params_;
};

}  // namespace beluga

#endif

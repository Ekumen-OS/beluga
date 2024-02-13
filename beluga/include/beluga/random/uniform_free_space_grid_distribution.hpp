// Copyright 2024 Ekumen, Inc.
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

#ifndef BELUGA_RANDOM_UNIFORM_GRID_DISTRIBUTION_HPP
#define BELUGA_RANDOM_UNIFORM_GRID_DISTRIBUTION_HPP

#include <random>

#include <range/v3/utility/random.hpp>
#include <sophus/se2.hpp>

#include <beluga/sensor/data/occupancy_grid.hpp>

namespace beluga {

/// Primary template for a uniform grid distribution.
/**
 * \tparam T The result type for sampling from the distribution.
 */
template <class T>
class UniformFreeSpaceGridDistribution;

/// Specialization of uniform grid distribution for Sophus::SE2d.
template <>
class UniformFreeSpaceGridDistribution<Sophus::SE2d> {
 public:
  /// Constructs a uniform grid distribution based on the provided occupancy grid.
  /**
   * \tparam Derived A type derived from BaseOccupancyGrid2.
   * \param grid The occupancy grid from which free states will be computed.
   */
  template <class Derived>
  constexpr explicit UniformFreeSpaceGridDistribution(const BaseOccupancyGrid2<Derived>& grid)
      : free_states_{compute_free_states(static_cast<const Derived&>(grid))},
        distribution_{0, free_states_.size() - 1} {
    assert(free_states_.size() > 0);
  }

  /// Generates a random 2D pose.
  /**
   * This function generates a random pose by sampling a random rotation
   * from SO2 space and a random translation from the precomputed free states
   * based on the provided occupancy grid.
   *
   * \tparam URNG The type of the random number generator.
   * \param engine The random number generator engine.
   * \return A random Sophus::SE2d pose.
   */
  template <class URNG>
  [[nodiscard]] Sophus::SE2d operator()(URNG& engine) {
    return {Sophus::SO2d::sampleUniform(engine), free_states_[distribution_(engine)]};
  }

 private:
  std::vector<Eigen::Vector2d> free_states_;                 ///< Vector containing free states.
  std::uniform_int_distribution<std::size_t> distribution_;  ///< Uniform distribution for indices.

  template <class Grid>
  static std::vector<Eigen::Vector2d> compute_free_states(const Grid& grid) {
    return grid.coordinates_for(grid.free_cells(), Grid::Frame::kGlobal) | ranges::to<std::vector>;
  }
};

/// Deduction guide for 2D occupancy grids.
template <class Derived>
UniformFreeSpaceGridDistribution(const BaseOccupancyGrid2<Derived>&) -> UniformFreeSpaceGridDistribution<Sophus::SE2d>;

}  // namespace beluga

#endif

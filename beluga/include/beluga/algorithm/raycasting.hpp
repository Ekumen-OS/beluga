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

#ifndef BELUGA_ALGORITHM_RAYCASTING_HPP
#define BELUGA_ALGORITHM_RAYCASTING_HPP

#include <algorithm>
#include <optional>
#include <utility>

#include <beluga/algorithm/raycasting/bresenham.hpp>

#include <range/v3/view/all.hpp>
#include <range/v3/view/take_while.hpp>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

/**
 * \file
 * \brief Implementation of raycasting algorithms.
 */

namespace beluga {

/// Castable 2D ray.
/**
 * \tparam OccupancyGrid A 2D occupancy grid
 * \tparam Algorithm A callable type, taking start and end
 *   grid cells for a ray and returning the full trace.
 */
template <class OccupancyGrid, typename Algorithm = Bresenham2i>
class Ray2d {
 public:
  /// Constructs 2D ray with default ray tracing algorithm.
  /**
   * See Ray2d(const OccupancyGrid &, Algorithm, const Sophus::SE2d&, double)
   * for further reference on constructor arguments.
   */
  explicit Ray2d(const OccupancyGrid& grid, const Sophus::SE2d& source_pose, double max_range) noexcept
      : Ray2d(grid, Algorithm{}, source_pose, max_range) {}

  /// Constructs 2D ray with an specific ray tracing algorithm.
  /**
   * \param grid Grid on which to perform ray casting.
   * \param algorithm Ray tracing algorithm implementation.
   * \param source_pose Pose of the source of the ray in the
   *   same frame as that on which the `grid` origin is defined.
   * \param max_range Maximum range for the ray, in meters.
   */
  explicit Ray2d(
      const OccupancyGrid& grid,
      Algorithm algorithm,
      const Sophus::SE2d& source_pose,
      double max_range) noexcept
      : grid_(grid),
        algorithm_(std::move(algorithm)),
        source_pose_in_local_frame_(grid_.origin().inverse() * source_pose),
        source_cell_(grid_.cell_near(source_pose_in_local_frame_.translation())),
        max_range_(max_range) {}

  /// Computes ray trace along a given direction.
  /**
   * \param bearing Direction for ray tracing.
   * \return Full range of grid cells traced by the ray.
   *   That is, regardless of grid cells' state.
   */
  [[nodiscard]] auto trace(const Sophus::SO2d& bearing) const {
    // Don't do the multiplication in SE2, it's slower
    const auto& r1 = source_pose_in_local_frame_.so2();
    const auto& t1 = source_pose_in_local_frame_.translation();
    const auto t2 = bearing.unit_complex() * max_range_;
    const auto far_end_in_local_frame = (r1 * t2 + t1).eval();
    const auto far_end_cell = grid_.cell_near(far_end_in_local_frame);
    const auto cell_is_valid = [this](const auto& cell) { return grid_.contains(cell); };
    return algorithm_(source_cell_, far_end_cell) | ranges::views::take_while(cell_is_valid);
  }

  /// Casts ray along a given direction.
  /**
   * Distances are measured from cell centroid to cell centroid.
   *
   * \param bearing Direction for ray casting.
   * \return Distance in meters to first non free cell hit by the ray, if any.
   */
  [[nodiscard]] std::optional<double> cast(const Sophus::SO2d& bearing) const {
    for (const auto& cell : trace(bearing)) {
      if (!grid_.free_at(cell)) {
        const auto source_position = grid_.coordinates_at(source_cell_);
        const auto cell_position = grid_.coordinates_at(cell);
        const auto distance = (cell_position - source_position).norm();
        return std::make_optional(std::min(distance, max_range_));
      }
    }
    return std::nullopt;
  }

 private:
  const OccupancyGrid& grid_;
  const Algorithm algorithm_;
  const Sophus::SE2d source_pose_in_local_frame_;
  const Eigen::Vector2i source_cell_;
  const double max_range_;
};

}  // namespace beluga

#endif

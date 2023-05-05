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
 * \tparam Grid A 2D grid
 * \tparam Algorithm A callable type, taking start and end
 *   grid cells for a ray and returning the full trace.
 */
template <class Grid, typename Algorithm = Bresenham2i>
class Ray2d {
 public:
  /// Constructs 2D ray with default ray tracing algorithm.
  /**
   * See Ray2d(const Grid &, Algorithm, const Sophus::SE2d&, double)
   * for further reference on constructor arguments.
   */
  explicit Ray2d(const Grid& grid, const Sophus::SE2d& source_pose, double max_range) noexcept
      : Ray2d(grid, Algorithm{}, source_pose, max_range) {}

  /// Constructs 2D ray with an specific ray tracing algorithm.
  /**
   * \param grid Grid on which to perform ray casting.
   * \param algorithm Ray tracing algorithm implementation.
   * \param source_pose Pose of the source of the ray in the
   *   same frame as that on which the `grid` origin is defined.
   * \param max_range Maximum range for the ray, in meters.
   */
  explicit Ray2d(const Grid& grid, Algorithm algorithm, const Sophus::SE2d& source_pose, double max_range) noexcept
      : grid_(grid),
        algorithm_(std::move(algorithm)),
        source_pose_in_grid_frame_(grid_.origin().inverse() * source_pose),
        max_range_(max_range) {}

  /// Computes ray trace along a given direction.
  /**
   * \param bearing Direction for ray tracing.
   * \return Full range of grid cells traced by the ray.
   *   That is, regardless of grid cells' state.
   */
  [[nodiscard]] auto trace(const Sophus::SO2d& bearing) const {
    const auto far_end_pose_in_source_frame = Sophus::SE2d{
        Sophus::SO2d{0.},
        Eigen::Vector2d{max_range_ * bearing.unit_complex().x(), max_range_ * bearing.unit_complex().y()}};
    const auto far_end_pose_in_grid_frame = source_pose_in_grid_frame_ * far_end_pose_in_source_frame;
    const auto start_cell = grid_.cell(source_pose_in_grid_frame_.translation());
    const auto end_cell = grid_.cell(far_end_pose_in_grid_frame.translation());
    const auto cell_is_valid = [this](const auto& cell) { return grid_.valid(cell); };
    return algorithm_(start_cell, end_cell) | ranges::views::take_while(cell_is_valid);
  }

  /// Casts ray along a given direction.
  /**
   * Distances are measured from cell centroid to cell centroid.
   *
   * \param bearing Direction for ray casting.
   * \return Distance in meters to first non free cell hit by the ray, if any.
   */
  [[nodiscard]] std::optional<double> cast(const Sophus::SO2d& bearing) const {
    const auto is_free = [this](const auto& cell) {
      // TODO(hidmic): move to occupancy grid API
      return Grid::Traits::is_free(grid_.data()[grid_.index(cell)]);
    };
    for (const auto& cell : trace(bearing)) {
      if (!is_free(cell)) {
        const auto start_cell = grid_.cell(source_pose_in_grid_frame_.translation());
        const auto distance = (grid_.point(cell) - grid_.point(start_cell)).norm();
        return std::make_optional(std::min(distance, max_range_));
      }
    }
    return std::nullopt;
  }

 private:
  const Grid& grid_;
  const Algorithm algorithm_;
  const Sophus::SE2d source_pose_in_grid_frame_;
  const double max_range_;
};

}  // namespace beluga

#endif

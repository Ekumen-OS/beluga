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

#ifndef BELUGA_SENSOR_DATA_DENSE_GRID2_MIXIN_HPP
#define BELUGA_SENSOR_DATA_DENSE_GRID2_MIXIN_HPP

#include <optional>
#include <vector>

#include <Eigen/Core>

/**
 * \file
 * \brief Concepts and abstract implementations of dense grids.
 */

namespace beluga {

/**
 * \page DenseGrid2Page Beluga named requirements: DenseGrid2
 *
 * Dense grids support random, indexed access to each cell. These
 * grids have a finite extent and thus can hold cell data in finite
 * memory. Dense grids are regular grids, meaning they satisfy
 * \ref RegularGrid2Page requirements.
 *
 * A type `G` satisfies `DenseGrid2` requirements if it satisfies
 * \ref RegularGrid2Page and given `g` a possibly const instance of `G`:
 * - given possibly const grid cell coordinates `xi` and `yi` of type `int`,
 *   `g.contains(xi, yi)` checks whether such cell is included in the grid;
 * - given possibly const grid cell coordinates `pi` of `Eigen::Vector2i` type,
 *   `g.contains(p)` checks whether such cell is included in the grid;
 * - given possibly const grid cell coordinates `pi` of `Eigen::Vector2i` type,
 *   `g.data_at(p)` optionally returns cell data, if cell is included;
 * - given possibly const embedding space coordinates `x` and `y` of type `double`,
 *   `g.data_near(x, y)` optionally returns cell data, if cell is included;
 * - given possibly const embedding space coordinates `p` of `Eigen::Vector2d` type,
 *   `g.data_near(p)` optionally returns cell data, if cell is included;
 * - given possibly const grid cell coordinates `xi` and `yi` of type `int`,
 *   `g.neighborhood4(xi, yi)` computes the cell 4-connected neighborhood as
 *   a range of `Eigen::Vector2i` type;
 * - given possibly const grid cell coordinates `pi` of `Eigen::Vector2i` type,
 *   `g.neighborhood4(p)` computes the cell 4-connected neighborhood as a
 *   range of `Eigen::Vector2i` type.
 */

/// Dense 2D grid base type.
/**
 * \tparam Mixin Mixin type
 */
template <typename Mixin>
class DenseGrid2Mixin : public Mixin {
 public:
  /// @brief Mixin constructor
  /// @param ...args arguments to be forwarded to other mixins in the chain
  template <typename... Args>
  explicit DenseGrid2Mixin(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Checks if a cell is included in the grid.
  /**
   * \param xi Grid cell x-axis coordinate.
   * \param yi Grid cell y-axis coordinate.
   */
  [[nodiscard]] bool contains(int xi, int yi) const {
    const auto width = static_cast<int>(this->self().width());
    const auto height = static_cast<int>(this->self().height());
    return xi >= 0 && yi >= 0 && xi < width && yi < height;
  }

  /// Checks if a cell is included in the grid.
  /**
   * \param pi Grid cell coordinates.
   */
  [[nodiscard]] bool contains(const Eigen::Vector2i& pi) const { return this->self().contains(pi.x(), pi.y()); }

  /// Gets cell data, if included.
  /**
   * \param pi Grid cell coordinates.
   * \return Cell data if included, `std::nullopt` otherwise.
   */
  [[nodiscard]] auto data_at(const Eigen::Vector2i& pi) const { return this->self().data_at(pi.x(), pi.y()); }

  /// Gets nearest cell data, if included.
  /**
   * \param x Plane x-axis coordinate.
   * \param y Plane y-axis coordinate.
   * \return Cell data if included, `std::nullopt` otherwise.
   */
  [[nodiscard]] auto data_near(double x, double y) const { return this->self().data_at(this->self().cell_near(x, y)); }

  /// Gets nearest cell data, if included.
  /**
   * \param p Plane coordinates.
   * \return Cell data if included, `std::nullopt` otherwise.
   */
  [[nodiscard]] auto data_near(const Eigen::Vector2d& p) const { return this->self().data_near(p.x(), p.y()); }

  /// Computes 4-connected neighborhood for cell.
  /**
   * \param xi Grid cell x-axis coordinate.
   * \param yi Grid cell y-axis coordinate.
   * \return range of neighbor cells.
   */
  [[nodiscard]] auto neighborhood4(int xi, int yi) const {
    auto result = std::vector<Eigen::Vector2i>{};
    result.reserve(4);
    if (xi < static_cast<int>(this->self().width() - 1)) {
      result.emplace_back(xi + 1, yi);
    }
    if (yi < static_cast<int>(this->self().height() - 1)) {
      result.emplace_back(xi, yi + 1);
    }
    if (xi > 0) {
      result.emplace_back(xi - 1, yi);
    }
    if (yi > 0) {
      result.emplace_back(xi, yi - 1);
    }
    return result;
  }

  /// Computes 4-connected neighborhood for cell.
  /**
   * \param pi Grid cell coordinates.
   * \return range of neighbor cells.
   */
  [[nodiscard]] auto neighborhood4(const Eigen::Vector2i& pi) const {
    return this->self().neighborhood4(pi.x(), pi.y());
  }
};

}  // namespace beluga

#endif

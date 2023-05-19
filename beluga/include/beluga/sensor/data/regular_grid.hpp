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

#ifndef BELUGA_SENSOR_DATA_REGULAR_GRID_HPP
#define BELUGA_SENSOR_DATA_REGULAR_GRID_HPP

#include <cmath>

#include <ciabatta/ciabatta.hpp>

#include <range/v3/view/transform.hpp>

#include <Eigen/Core>

/**
 * \file
 * \brief Concepts and abstract implementations of regular grids.
 */

namespace beluga {

/**
 * \page RegularGrid2Page Beluga named requirements: RegularGrid2
 *
 * Regular grids divide space in evenly sized portions or _cells_. A coordinate
 * system over integers can thus be defined, in addition to that of the space
 * in which the grid is embedded.
 *
 * A type `G` satisfies `RegularGrid2` requirements if given `g` a possibly
 * const instance of `G`:
 * - `g.resolution()` returns the side length of all grid cells as a `double`;
 * - given possibly const embedding space coordinates `x` and `y` of type `double`,
 *   `g.cell_at(x, y)` returns grid cell coordinates as an `Eigen::Vector2d` value;
 * - given possibly const embedding space coordinates `p` of `Eigen::Vector2d` type,
 *   `g.cell_at(p)` returns grid cell coordinates as an `Eigen::Vector2d` value;
 * - given possibly const grid cell coordinates `xi` and `yi` of type `int`,
 *   `g.coordinates_at(xi, yi)` returns embedding space coordinates as an
 *   `Eigen::Vector2d` value;
 * - given possibly const grid cell coordinates `pi` of `Eigen::Vector2i` type,
 *   `g.coordinates_at(p)` returns embedding space coordinates as an
 *   `Eigen::Vector2d` value;
 * - given a possibly const range `r` of grid cell coordinates of `Eigen::Vector2i`
 *   type, `g.coordinates_for(r)` returns a range of embedding space coordinates as
 *   `Eigen::Vector2d` values.
 */

/// Regularly spaced 2D grid base type.
/**
 * When instantiated, it satisfies \ref RegularGrid2Page.
 *
 * \tparam Derived Concrete regular grid type. It must define
 * `Derived::resolution()`, as described in \ref RegularGrid2Page.
 */
template <typename Derived>
class BaseRegularGrid2 : public ciabatta::ciabatta_top<Derived> {
 public:
  /// Compute nearest grid cell coordinates given plane coordinates.
  /**
   * Note this is a surjective function.
   *
   * \param x Plane x-axis coordinate.
   * \param y Plane y-axis coordinate.
   * \return Grid cell coordinates.
   */
  [[nodiscard]] Eigen::Vector2i cell_near(double x, double y) const {
    const auto xi = static_cast<int>(std::floor(x / this->self().resolution()));
    const auto yi = static_cast<int>(std::floor(y / this->self().resolution()));
    return Eigen::Vector2i{xi, yi};
  }

  /// Compute nearest grid cell coordinates given plane coordinates.
  /**
   * Note this is a surjective function.
   *
   * \param p Plane coordinates.
   * \return Grid cell coordinates.
   */
  [[nodiscard]] Eigen::Vector2i cell_near(const Eigen::Vector2d& p) const {
    return this->self().cell_near(p.x(), p.y());
  }

  /// Compute plane coordinates given grid cell coordinates.
  /**
   * Note this is an injective function.
   *
   * \param xi Grid cell x-axis coordinate.
   * \param yi Grid cell y-axis coordinate.
   * \return Plane coordinates of the cell centroid.
   */
  [[nodiscard]] Eigen::Vector2d coordinates_at(int xi, int yi) const {
    return Eigen::Vector2d{
        (static_cast<double>(xi) + 0.5) * this->self().resolution(),
        (static_cast<double>(yi) + 0.5) * this->self().resolution()};
  }

  /// Compute plane coordinates given grid cell coordinates.
  /**
   * Note this is an injective function.
   *
   * \param pi Grid cell coordinates.
   * \return Plane coordinates of the cell centroid.
   */
  [[nodiscard]] Eigen::Vector2d coordinates_at(const Eigen::Vector2i& pi) const {
    return this->self().coordinates_at(pi.x(), pi.y());
  }

  /// Compute plane coordinates given a range of cell coordinates.
  /**
   * \param cells Range of grid cell identifiers.
   * \return Range of plane coordinates.
   */
  template <class Range>
  [[nodiscard]] auto coordinates_for(Range&& cells) const {
    return cells | ranges::views::transform([this](const auto& cell) { return this->self().coordinates_at(cell); });
  }
};

}  // namespace beluga

#endif

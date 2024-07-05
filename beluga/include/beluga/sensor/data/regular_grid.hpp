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
#include <beluga/eigen_compatibility.hpp>

/**
 * \file
 * \brief Concepts and abstract implementations of regular grids.
 */

namespace beluga {

/**
 * \page RegularGridPage Beluga named requirements: RegularGrid
 *
 * Regular grids divide space in evenly sized portions or _cells_. A coordinate
 * system over integers can thus be defined, in addition to that of the space
 * in which the grid is embedded.
 *
 * A type `G` satisfies `RegularGrid` requirements if given `g`, a possibly
 * const instance of `G`:
 * - `g.resolution()` returns the side length of all grid N-dimensional cells as a `double`;
 * - given possibly const embedding space coordinates `x` of type `Eigen::Vector<double, NDim>`,
 *   `g.cell_at(x)` returns grid cell coordinates as an `Eigen::Vector<int, NDim1>` value;
 * - given possibly const embedding space coordinates `p` of `Eigen::Vector<double, NDim>` type,
 *   `g.cell_at(p)` returns grid cell coordinates as an `Eigen::Vector<int, NDim>` value;
 * - given possibly const grid cell coordinates `xi` of type `Eigen::Vector<int, NDim>`,
 *   `g.coordinates_at(xi)` returns embedding space coordinates as an
 *   `Eigen::Vector<double, NDim>` value;
 * - given a possibly const range `r` of grid cell coordinates of `Eigen::Vector<int, NDim>`
 *   type, `g.coordinates_for(r)` returns a range of embedding space coordinates as
 *   `Eigen::Vector<double, NDim>` values.
 */

/// Regularly spaced N dimensional grid base type.
/**
 * When instantiated, it satisfies \ref RegularGridPage.
 *
 * \tparam Derived Concrete regular grid type. It must define
 * `Derived::resolution()`, as described in \ref RegularGridPage.
 * \tparam NDim Dimension of the grid.
 */
template <typename Derived, int NDim>
class BaseRegularGrid : public ciabatta::ciabatta_top<Derived> {
 public:
  /// Compute nearest grid cell coordinates given plane coordinates.
  /**
   * Note this is a surjective function.
   *
   * \param p Plane coordinates.
   * \return Grid cell coordinates.
   */
  [[nodiscard]] Eigen::Vector<int, NDim> cell_near(const Eigen::Vector<double, NDim>& p) const {
    const auto inv_resolution = 1. / this->self().resolution();
    return (p * inv_resolution).array().floor().template cast<int>();
  }

  /// Compute plane coordinates given grid cell coordinates.
  /**
   * Note this is an injective function.
   *
   * \param pi Grid cell coordinates.
   * \return Plane coordinates of the cell centroid.
   */
  [[nodiscard]] Eigen::Vector<double, NDim> coordinates_at(const Eigen::Vector<int, NDim>& pi) const {
    return (pi.template cast<double>() + Eigen::Vector<double, NDim>::Ones() * 0.5) * this->self().resolution();
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
/// Convenience alias for a 2D base regular grid.
template <typename Derived>
using BaseRegularGrid2 = BaseRegularGrid<Derived, 2>;

/// Convenience alias for a 3D base regular grid.
template <typename Derived>
using BaseRegularGrid3 = BaseRegularGrid<Derived, 3>;

}  // namespace beluga

#endif

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

#ifndef BELUGA_SENSOR_DATA_LINEAR_GRID_HPP
#define BELUGA_SENSOR_DATA_LINEAR_GRID_HPP

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

/**
 * \file
 * \brief Concepts and abstract implementations of linear (ie. contiguous storage) grids.
 */

namespace beluga {

/**
 * \page LinearGrid2Page Beluga named requirements: LinearGrid2
 *
 * Linear grids use contiguous memory layouts, and thus afford integer
 * indexes. Linear grids are dense grids, meaning they satisfy
 * \ref DenseGrid2Page requirements.
 *
 * A type `G` satisfies `LinearGrid2` requirements if it satisfies
 * \ref DenseGrid2Page and given `g` a possibly const instance of `G`:
 * - `g.data()` returns a possibly const reference to an indexable,
 *   random access linear data structure containing grid cell data values;
 * - given possibly const grid cell index `i` of `std::size_t` type,
 *   `g.data_at(i)` optionally returns cell data, if cell is included;
 * - given possibly const grid cell index `i` of `std::size_t` type,
 *   `g.coordinates_at(i)` returns embedding space coordinates as an
 *   `Eigen::Vector2d` value;
 * - given possibly const grid cell index `i` of `std::size_t` type,
 *   `g.neighborhood4(i)` computes the cell 4-connected neighborhood as a
 *   range of `std::size_t` indices.
 */

/// Linear 2D grid base type.
/**
 * When instantiated, it satisfies \ref LinearGrid2Page.
 *
 * \tparam Derived Concrete linear grid type. It must define
 * `Derived::width()`, `Derived::height()`, `Derived::resolution()`,
 * `Derived::data_at(std::size_t)`, `Derived::index_at(int, int)`,
 * and `Derived::data()` as described in \ref LinearGrid2Page.
 */
template <typename Mixin>
class BaseLinearGrid2Mixin : public Mixin {
 public:
  /// @brief Mixin constructor
  /// @param ...args arguments to be forwarded to other mixins in the chain
  template <typename... Args>
  explicit BaseLinearGrid2Mixin(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Computes index for given grid cell coordinates.
  /*
   * \param xi Grid cell x-axis coordinate.
   * \param yi Grid cell y-axis coordinate.
   */
  // RENAME INDEX AS CELL_ID TO AVOID AMBIGUITIES
  [[nodiscard]] std::size_t index_at(int xi, int yi) const {
    return static_cast<std::size_t>(yi) * this->self().width() + static_cast<std::size_t>(xi);
  }

  /// Computes index for given grid cell coordinates.
  /**
   * \param pi Grid cell coordinates.
   */
  [[nodiscard]] std::size_t index_at(const Eigen::Vector2i& pi) const { return this->self().index_at(pi.x(), pi.y()); }

  using Mixin::data_at;

  /// Gets cell data, if included.
  /**
   * This is not the proper place for this, but I need to pivot.
   * \param xi Grid cell x-axis coordinate.
   * \param yi Grid cell y-axis coordinate.
   * \return Cell data if included, `std::nullopt` otherwise.
   */
  [[nodiscard]] auto data_at(int xi, int yi) const {
    return this->self().contains(xi, yi) ? std::make_optional(this->self().data()[this->self().index_at(xi, yi)])
                                         : std::nullopt;
  }
};

}  // namespace beluga

#endif

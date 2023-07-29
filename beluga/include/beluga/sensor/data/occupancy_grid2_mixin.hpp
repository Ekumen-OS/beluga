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

#ifndef BELUGA_SENSOR_DATA_OCCUPANCY_GRID2_MIXIN_HPP
#define BELUGA_SENSOR_DATA_OCCUPANCY_GRID2_MIXIN_HPP

#include <cstdint>
#include <tuple>
#include <vector>

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>

#include <Eigen/Core>

/**
 * \file
 * \brief Concepts and abstract implementations of occupancy grids.
 */

namespace beluga {

/**
 * \page OccupancyGrid2Page Beluga named requirements: OccupancyGrid2d
 *
 * Occupancy grids model obstacle probability. These grids also define
 * their own frame in the grid embedding space, adding so called global
 * coordinates to regular (aka local) coordinates.
 *
 * A type `G` satisfies `OccupancyGrid2d` requirements if it satisfies
 * that given `g` a possible const instance of `G`:
 * - given possibly const grid cell coordinates `xi` and `yi` of type `int`,
 *   `g.free_at(xi, yi)` returns true if such cell is free;
 * - given possibly const grid cell coordinates `pi` of `Eigen::Vector2i` type,
 *   `g.free_at(p)` returns true if such cell is free;
 * - given possibly const embedding space coordinates `x` and `y` of type `double`,
 *   `g.free_near(x, y)` returns true if the nearest cell is free;
 * - given possibly const embedding space coordinates `p` of `Eigen::Vector2d` type,
 *   `g.free_near(p)` returns true if the nearest cell is free;
 * - given a possibly const range `r` of grid cell coordinates or indices and frame `f`,
 *   `g.coordinates_for(r, f)` returns a range of embedding space coordinates in the
 *   corresponding frame as `Eigen::Vector2d` values;
 * - `g.free_cells()` returns a range of `std::size_t` indices to free grid cells;
 * - `g.obstacle_data()` returns a range of `bool` values, representing grid cell occupancy;
 */

/// Occupancy 2D grid base type.
/**
 * \tparam Mixin Mixin type
 */
template <typename Mixin>
class OccupancyGrid2Mixin : public Mixin {
 public:
  /// Coordinate frames.
  enum class Frame { kLocal, kGlobal };

  /// @brief Mixin constructor
  /// @param ...args arguments to be forwarded to other mixins in the chain
  template <typename... Args>
  explicit OccupancyGrid2Mixin(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Checks if cell is free.
  /**
   * Note cells not included in the grid are non-free too.
   *
   * \param xi Grid cell x-axis coordinate.
   * \param yi Grid cell y-axis coordinate.
   */
  [[nodiscard]] bool free_at(int xi, int yi) const {
    const auto data = this->self().data_at(xi, yi);
    if (!data.has_value()) {
      return false;
    }
    return this->self().value_traits().is_free(data.value());
  }

  /// Checks if cell is free.
  /**
   * Note cells not included in the grid are non-free too.
   *
   * \param pi Grid cell coordinates.
   */
  [[nodiscard]] bool free_at(const Eigen::Vector2i& pi) const { return this->self().free_at(pi.x(), pi.y()); }

  /// Checks if nearest cell is free.
  /**
   * Note cells not included in the grid are non-free too.
   *
   * \param x Plane x-axis coordinate.
   * \param y Plane y-axis coordinate.
   */
  [[nodiscard]] bool free_near(double x, double y) const { return this->self().free_at(this->self().cell_near(x, y)); }

  /// Checks if nearest cell is free.
  /**
   * Note cells not included in the grid are non-free too.
   *
   * \param p Plane coordinates.
   */
  [[nodiscard]] bool free_near(const Eigen::Vector2d& p) const { return this->self().free_near(p.x(), p.y()); }

  using Mixin::coordinates_at;

  /// Compute plane coordinates given grid cell coordinates.
  /**
   * \param cell Grid cell coordinate.
   * \param frame Plane coordinate frame.
   * \return Plane coordinates in the corresponding `frame`.
   */
  [[nodiscard]] auto coordinates_at(const Eigen::Vector2i& cell, Frame frame) const {
    auto position = this->self().coordinates_at(cell);
    if (frame == Frame::kGlobal) {
      position = this->self().origin() * position;
    }
    return position;
  }

  /// Compute plane coordinates for a range of grid cells.
  /**
   * \param cells Range of grid integer coordinates.
   * \param frame Plane coordinate frame.
   * \return Range of plane coordinates in the corresponding `frame`.
   */
  template <class Range>
  [[nodiscard]] auto coordinates_for(Range&& cells, Frame frame) const {
    const auto get_coordinates = [this, frame](const auto& cell) { return this->self().coordinates_at(cell, frame); };
    return cells | ranges::views::transform(get_coordinates);
  }

  /// Retrieves a range containing the ids of free cells.
  [[nodiscard]] auto free_cells() const {
    const auto is_free = [value_traits = this->self().value_traits()](const auto& tuple) {
      return value_traits.is_free(std::get<1>(tuple));
    };
    const auto get_cell_id = [](const auto& tuple) { return std::get<0>(tuple); };
    return this->self().row_major_data() | ranges::views::filter(is_free) | ranges::views::transform(get_cell_id);
  }

  /// Retrieves a range containing the cells that are obstacles only.
  [[nodiscard]] auto obstacle_data() const {
    const auto is_occupied = [value_traits = this->self().value_traits()](const auto& tuple) {
      return value_traits.is_occupied(std::get<1>(tuple));
    };
    const auto get_cell_id = [](const auto& tuple) { return std::get<0>(tuple); };
    return this->self().row_major_data() | ranges::views::filter(is_occupied) | ranges::views::transform(get_cell_id);
  }

  using Mixin::data_at;

  /// Gets cell data, if included.
  /**
   * \param xi Grid cell x-axis coordinate.
   * \param yi Grid cell y-axis coordinate.
   * \return Cell data if included, `std::nullopt` otherwise.
   */
  [[nodiscard]] auto data_at(int xi, int yi) const {
    return this->self().contains(xi, yi) ? std::make_optional(this->self().cell(xi, yi)) : std::nullopt;
  }

 private:
  /// delivers a range of tuples with cell and cell value
  [[nodiscard]] auto row_major_data() const {
    const auto width = this->self().width();
    const auto height = this->self().height();
    const auto cell_count = width * height;
    const auto to_raster_data = [this, width](std::size_t raster_index) {
      const auto xi = static_cast<int>(raster_index % width);
      const auto yi = static_cast<int>(raster_index / width);
      const auto cell = Eigen::Vector2i(xi, yi);
      return std::make_tuple(cell, this->self().data_at(cell).value());
    };
    return ranges::views::iota(0, static_cast<int>(cell_count)) | ranges::views::transform(to_raster_data);
  }
};

}  // namespace beluga

#endif

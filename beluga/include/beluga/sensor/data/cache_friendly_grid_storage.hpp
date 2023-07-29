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

#ifndef BELUGA_SENSOR_DATA_BELUGA_CACHE_FRIENDLY_GRID_STORAGE_HPP
#define BELUGA_SENSOR_DATA_BELUGA_CACHE_FRIENDLY_GRID_STORAGE_HPP

#include <cstdint>
#include <iostream>
#include <memory>

namespace beluga {

/// @brief A cache friendly grid storage that efficiently packs grid patches in cache lines. This algorithm maps cache
/// lines to square tiles on the grid.
/// @tparam T Type of the data to be stored.
/// @tparam LineLenght Length of the minimum cache line to optimize for, in bytes.
template <typename T, std::size_t LineLenght>
class CacheFriendlyGridStorage {
 public:
  /// @brief Type of the data stored in the grid.
  using cell_type = T;

  /// @brief Constructs a map with the given initial values.
  /// @param width Width of the grid.
  /// @param height Height of the grid.
  /// @param init_values Initial contents of the map, in row-major order.
  CacheFriendlyGridStorage(std::size_t width, std::size_t height, std::initializer_list<T> init_values = {})
      : tile_cols_(ceil_div(width, tile_side_)),
        tile_rows_(ceil_div(height, tile_side_)),
        storage_(new(std::align_val_t{LineLenght}) T[tile_cols_ * tile_rows_ * tile_size_]),
        grid_width_(width),
        grid_height_(height) {
    static_assert(LineLenght % sizeof(T) == 0, "Line length must be a multiple of the data type size");
    static_assert(LineLenght >= sizeof(T), "Line length must be greater than the data type size");
    static_assert(is_power_of_two(LineLenght), "Line length must be a power of two");
    std::size_t index = 0;
    for (const auto& cell_value : init_values) {
      const auto x = static_cast<int>(index % grid_width_);
      const auto y = static_cast<int>(index / grid_width_);
      cell(x, y) = cell_value;
      ++index;
    }
  }

  /// @brief Access to the cell at the given coordinates.
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @return Reference to the cell at the given coordinates.
  [[nodiscard]] auto& cell(int x, int y) { return storage_[map_coordinates_to_index(x, y)]; }

  /// @brief Reading access to the cell at the given coordinates.
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @return Const reference to the cell at the given coordinates.
  [[nodiscard]] const auto& cell(int x, int y) const { return storage_[map_coordinates_to_index(x, y)]; }

  /// @brief Returns the width of the map (number of cells).
  [[nodiscard]] auto width() const { return grid_width_; }

  /// @brief Returns the height of the map (number of cells).
  [[nodiscard]] auto height() const { return grid_height_; }

  /// @brief Returns the raw contents of the internal buffer. For testing only.
  [[nodiscard]] auto& raw_data() { return storage_; }

 private:
  static constexpr bool is_power_of_base(std::size_t n, std::size_t base) {
    if (n < 1) {
      return false;
    }
    return (n == 1) || ((n % base == 0) && is_power_of_base(n / base, base));
  }

  static constexpr bool is_power_of_two(std::size_t n) { return is_power_of_base(n, 2); }

  static constexpr bool is_power_of_four(std::size_t n) { return is_power_of_base(n, 4); }

  static constexpr bool is_a_square_power_of_two(std::size_t n) {
    // if it's a power of four, it's also a perfect square
    return is_power_of_four(n);
  }

  static constexpr std::size_t calculate_tile_area(std::size_t elements_per_line) {
    if (is_a_square_power_of_two(elements_per_line)) {
      // the elements in a line can be mapped to a square tile (4, 16, 64,
      // etc) use a single line for each square tile
      return elements_per_line;
    } else {
      // the elements in a line can't be mapped to a square tile (2, 8, 32, etc), so
      // we use two consecutive lines for each tile
      return elements_per_line * 2;
    }
  }

  static constexpr std::size_t calculate_tile_side(std::size_t elements_per_line) {
    auto tile_area = calculate_tile_area(elements_per_line);
    // we calculate the side length by calculating the square root base four,
    // and elevating two to that power
    std::size_t tile_side = 1;
    while (tile_area > 1) {
      tile_area /= 4;
      tile_side *= 2;
    }
    return tile_side;
  }

  static constexpr std::size_t ceil_div(std::size_t numerator, std::size_t denominator) {
    return ((numerator + denominator - 1) / denominator);
  }

  std::size_t mapping_function(std::size_t x, std::size_t y) const {
    if constexpr (!is_a_square_power_of_two(LineLenght / sizeof(T))) {
      // full formula for rectangular tiles
      const auto tile_x = x / tile_side_;
      const auto tile_y = y / tile_side_;
      auto cell_x = x % tile_side_;
      auto cell_y = y % tile_side_;

      // if we are using two lines per tile, we need to swap the coordinates
      // in alternating tiles, to avoid large scale anisotropy
      if (tile_x % 2 == tile_y % 2) {
        std::swap(cell_x, cell_y);
      }

      const auto tile_index = (tile_y * tile_cols_ + tile_x) * tile_size_;
      const auto cell_index = cell_y * tile_side_ + cell_x;
      return tile_index + cell_index;

    } else {
      // fast-track formula for square tiles (the performance difference with the one above is
      // about 10% when used for square tiles)
      auto cell_x = x % tile_side_;
      auto cell_y = y % tile_side_;
      return ((y - cell_y) * tile_cols_ + cell_y + x - cell_x) * tile_side_ + cell_x;
    }
  }

  constexpr std::size_t map_coordinates_to_index(int x, int y) const { return mapping_function(x, y); }

  static constexpr std::size_t tile_size_{calculate_tile_area(LineLenght / sizeof(T))};
  static constexpr std::size_t tile_side_{calculate_tile_side(LineLenght / sizeof(T))};

  const std::size_t tile_cols_;
  const std::size_t tile_rows_;
  std::unique_ptr<T[]> storage_;

  const std::size_t grid_width_;
  const std::size_t grid_height_;
};

}  // namespace beluga

#endif

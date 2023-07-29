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

#ifndef BELUGA_SENSOR_DATA_CACHE_FRIENDLY_GRID_STORAGE_4_HPP
#define BELUGA_SENSOR_DATA_CACHE_FRIENDLY_GRID_STORAGE_4_HPP

#include <cstdint>
#include <iostream>
#include <memory>

namespace beluga {

/// @brief A cache friendly grid storage that efficiently packs grid patches in cache lines. This algorithm maps cache
/// lines to square tiles, and also groups tiles in 2x2 groups. The tiles are transposed in a checkerboard fashion to
/// try to exploit cache prefecth algorithms in the cpu.
/// @tparam T Type of the data to be stored.
/// @tparam LineLenght Length of the minimum cache line to optimize for, in bytes.
template <typename T, std::size_t LineLenght>
class CacheFriendlyGridStorage4 {
 public:
  /// @brief Type of the data stored in the grid.
  using cell_type = T;

  /// @brief Constructs a map with the given initial values.
  /// @param width Width of the grid.
  /// @param height Height of the grid.
  /// @param init_values Initial contents of the map, in row-major order.
  CacheFriendlyGridStorage4(std::size_t width, std::size_t height, std::initializer_list<T> init_values = {})
      : kL2Tilecols_(ceil_div(width, kL2TileSide)),
        kL2TileRows_(ceil_div(height, kL2TileSide)),
        storage_(new(std::align_val_t{LineLenght}) T[kL2Tilecols_ * kL2TileRows_ * kL2TileSize]),
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

 private:
  // NOLINTBEGIN
  [[nodiscard]] static constexpr bool is_power_of_base(std::size_t n, std::size_t base) {
    if (n < 1) {
      return false;
    }
    return (n == 1) || ((n % base == 0) && is_power_of_base(n / base, base));
  }
  // NOLINTEND

  [[nodiscard]] static constexpr bool is_power_of_two(std::size_t n) { return is_power_of_base(n, 2); }

  [[nodiscard]] static constexpr bool is_power_of_four(std::size_t n) { return is_power_of_base(n, 4); }

  [[nodiscard]] static constexpr bool is_a_square_power_of_two(std::size_t n) {
    // if it's a power of four, it's also a perfect square
    return is_power_of_four(n);
  }

  [[nodiscard]] static constexpr std::size_t calculate_tile_area(std::size_t elements_per_line) {
    if (is_a_square_power_of_two(elements_per_line)) {
      // the elements in a line can be mapped to a square tile (4, 16, 64,
      // etc) use a single line for each square tile
      return elements_per_line;
    }
    // the elements in a line can't be mapped to a square tile (2, 8, 32, etc), so
    // we use two consecutive lines for each tile
    return elements_per_line * 2;
  }

  [[nodiscard]] static constexpr std::size_t calculate_tile_side(std::size_t elements_per_line) {
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

  [[nodiscard]] static constexpr std::size_t ceil_div(std::size_t numerator, std::size_t denominator) {
    return ((numerator + denominator - 1) / denominator);
  }

  [[nodiscard]] std::size_t mapping_function(std::size_t x, std::size_t y) const {
    const auto l2_tile_x = x / kL2TileSide;
    const auto l2_tile_y = y / kL2TileSide;
    auto l2_cell_x = x % kL2TileSide;
    auto l2_cell_y = y % kL2TileSide;

    if ((l2_tile_x + l2_tile_y) & 0x01) {
      std::swap(l2_cell_x, l2_cell_y);
    }

    const auto l1_tile_x = l2_cell_x / kL1TileSide;
    const auto l1_tile_y = l2_cell_y / kL1TileSide;
    const auto l1_cell_x = l2_cell_x % kL1TileSide;
    const auto l1_cell_y = l2_cell_y % kL1TileSide;

    const auto l2_tile_offset = (l2_tile_y * kL2Tilecols_ + l2_tile_x) * kL2TileSize;
    const auto l1_tile_offset = (l1_tile_y * kL1TileCols + l1_tile_x) * kL1TileSize;
    const auto l1_cell_offset = l1_cell_y * kL1TileSide + l1_cell_x;
    return l2_tile_offset + l1_tile_offset + l1_cell_offset;
  }

  [[nodiscard]] constexpr std::size_t map_coordinates_to_index(int x, int y) const {
    return mapping_function(static_cast<std::size_t>(x), static_cast<std::size_t>(y));
  }

  static constexpr std::size_t kL1TileCols{2};
  static constexpr std::size_t kL1TileRows{2};

  static constexpr std::size_t kL2TileSize{kL1TileRows * kL1TileCols * calculate_tile_area(LineLenght / sizeof(T))};
  static constexpr std::size_t kL2TileSide{kL1TileCols * calculate_tile_side(LineLenght / sizeof(T))};
  static constexpr std::size_t kL1TileSize{calculate_tile_area(LineLenght / sizeof(T))};
  static constexpr std::size_t kL1TileSide{calculate_tile_side(LineLenght / sizeof(T))};

  const std::size_t kL2Tilecols_;
  const std::size_t kL2TileRows_;
  std::unique_ptr<T[]> storage_;  // NOLINT

  const std::size_t grid_width_;
  const std::size_t grid_height_;
};

}  // namespace beluga

#endif

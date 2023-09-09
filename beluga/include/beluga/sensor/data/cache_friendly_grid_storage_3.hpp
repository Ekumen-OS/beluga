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

#ifndef BELUGA_SENSOR_DATA_CACHE_FRIENDLY_GRID_STORAGE_3_HPP
#define BELUGA_SENSOR_DATA_CACHE_FRIENDLY_GRID_STORAGE_3_HPP

#include <cstdint>
#include <iostream>
#include <memory>

namespace beluga {

/// @brief A cache friendly grid storage that efficiently packs grid patches in cache lines. This algorithm transposes
/// the cells odd-numbered diagonals to equalize the effect of cache on both horizontal and vertical directions. For
/// highly rectangular grids, this algorithm is inefficient in terms of memory usage.
/// @tparam T Type of the data to be stored.
template <typename T>
class CacheFriendlyGridStorage3 {
 public:
  /// @brief Type of the data stored in the grid.
  using cell_type = T;

  /// @brief Constructs a map with the given initial values.
  /// @param width Width of the grid.
  /// @param height Height of the grid.
  /// @param init_values Initial contents of the map, in row-major order.
  CacheFriendlyGridStorage3(std::size_t width, std::size_t height, std::initializer_list<T> init_values = {})
      : storage_width_(next_even(std::max(width, height))),
        storage_height_(std::max(width, height)),
        storage_(storage_width_ * storage_height_, WrappedT{}),
        grid_width_(width),
        grid_height_(height) {
    std::size_t index = 0;
    for (const auto& cell_value : init_values) {
      const auto x = index % grid_width_;
      const auto y = index / grid_width_;
      cell_impl(x, y) = cell_value;
      ++index;
    }
  }

  /// @brief Access to the cell at the given coordinates.
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @return Reference to the cell at the given coordinates.
  [[nodiscard]] auto& cell(int x, int y) { return cell_impl(static_cast<std::size_t>(x), static_cast<std::size_t>(y)); }

  /// @brief Reading access to the cell at the given coordinates.
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @return Const reference to the cell at the given coordinates.
  [[nodiscard]] const auto& cell(int x, int y) const {
    return cell_impl(static_cast<std::size_t>(x), static_cast<std::size_t>(y));
  }

  /// @brief Returns the width of the map (number of cells).
  [[nodiscard]] auto width() const { return grid_width_; }

  /// @brief Returns the height of the map (number of cells).
  [[nodiscard]] auto height() const { return grid_height_; }

 private:
  // Wrap the type in a struct to avoid dealing with special cases for std::vector<bool>
  struct WrappedT {
    T value{};
  };

  const std::size_t storage_width_;
  const std::size_t storage_height_;

  std::vector<WrappedT> storage_;

  const std::size_t grid_width_;
  const std::size_t grid_height_;

  [[nodiscard]] static constexpr std::size_t next_even(std::size_t n) { return (n % 2 == 0) ? n : n + 1; }

  [[nodiscard]] auto& cell_impl(std::size_t x, std::size_t y) {
    // transpose odd-numbered diagonals. This mapping causes cache lines to map to the grid
    // like this:
    //
    // A C A G
    // A C E C
    // E C E G
    // A G E G
    //
    // instead of the nominal row-major order:
    //
    // A A A A
    // C C C C
    // E E E E
    // G G G G
    //
    // The aim is to tend to equalize the effect of cache on
    // both the horizontal and vertical directions.
    const auto transpose = (x + y) & 0x01;

    if (!transpose) {
      // straight
      return storage_[y * storage_width_ + x].value;
    }

    // transposed
    return storage_[x * storage_width_ + y].value;
  }

  [[nodiscard]] const auto& cell_impl(std::size_t x, std::size_t y) const {
    // transpose odd-numbered diagonals. See comment in non-const version.
    const auto transpose = (x + y) & 0x01;

    if (!transpose) {
      // straight
      return storage_[y * storage_width_ + x].value;
    }

    // transposed
    return storage_[x * storage_width_ + y].value;
  }
};

}  // namespace beluga

#endif

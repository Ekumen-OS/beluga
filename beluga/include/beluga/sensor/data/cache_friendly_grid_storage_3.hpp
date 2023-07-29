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

#ifndef BELUGA_SENSOR_DATA_BELUGA_CACHE_FRIENDLY_GRID_STORAGE_3_HPP
#define BELUGA_SENSOR_DATA_BELUGA_CACHE_FRIENDLY_GRID_STORAGE_3_HPP

#include <cstdint>
#include <iostream>
#include <memory>

namespace beluga {

/// @brief A cache friendly grid storage that efficiently packs grid patches in cache lines. This algorithm transposes
/// the cells odd-numbered diagonals to equalize the effect of cache on both horizontal and vertical directions.
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
        storage_(next_even(std::max(width, height)) * std::max(width, height), WrappedT{}),
        grid_width_(width),
        grid_height_(height) {
    std::size_t index = 0;
    for (const auto& cell_value : init_values) {
      const auto x = static_cast<int>(index % grid_width_);
      const auto y = static_cast<int>(index / grid_width_);
      cell_impl(x, y) = cell_value;
      ++index;
    }
  }

  /// @brief Access to the cell at the given coordinates.
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @return Reference to the cell at the given coordinates.
  [[nodiscard]] auto& cell(int x, int y) { return cell_impl(x, y); }

  /// @brief Reading access to the cell at the given coordinates.
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @return Const reference to the cell at the given coordinates.
  [[nodiscard]] const auto& cell(int x, int y) const { return cell_impl(x, y); }

  /// @brief Returns the width of the map (number of cells).
  [[nodiscard]] auto width() const { return grid_width_; }

  /// @brief Returns the height of the map (number of cells).
  [[nodiscard]] auto height() const { return grid_height_; }

 private:
  struct WrappedT {
    T value{};
  };

  const std::size_t storage_width_;
  const std::size_t storage_height_;

  std::vector<WrappedT> storage_;

  const std::size_t grid_width_;
  const std::size_t grid_height_;

  static constexpr std::size_t next_even(std::size_t n) { return (n % 2 == 0) ? n : n + 1; }

  auto& cell_impl(std::size_t x, std::size_t y) {
    const auto transpose = (x + y) & 0x01;
    if (!transpose) {
      // straight
      return storage_[y * storage_width_ + x].value;
    } else {
      // transposed
      return storage_[x * storage_width_ + y].value;
    }
  }

  const auto& cell_impl(std::size_t x, std::size_t y) const {
    const auto transpose = (x + y) & 0x01;
    if (!transpose) {
      // straight
      return storage_[y * storage_width_ + x].value;
    } else {
      // transposed
      return storage_[x * storage_width_ + y].value;
    }
  }
};

}  // namespace beluga

#endif

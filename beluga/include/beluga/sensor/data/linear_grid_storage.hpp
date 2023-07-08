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

#ifndef BELUGA_LINEAR_GRID_STORAGE_HPP
#define BELUGA_LINEAR_GRID_STORAGE_HPP

#include <cstdint>
#include <initializer_list>
#include <vector>

namespace beluga {

/// @brief A simple grid storage that stores the data in a vector.
/// @tparam T Type of the data to be stored.
template <typename T>
class LinearGridStorage {
 public:
  using cell_type = T;

  /// @brief Constructs a map with the given initial values.
  /// @param width Width of the grid.
  /// @param height Height of the grid.
  /// @param init_values Initial contents of the map, in row-major order.
  LinearGridStorage(std::size_t width, std::size_t height, std::initializer_list<T> init_values = {})
      : width_(width), height_(height), storage_(width * height, WrappedT{}) {
    const auto n = std::min(init_values.size(), storage_.size());
    for (std::size_t i = 0; i < n; ++i) {
      storage_[i].value = *(init_values.begin() + i);
    }
  }

  /// @brief Access to the cell at the given coordinates.
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @return Reference to the cell at the given coordinates.
  [[nodiscard]] auto& cell(int x, int y) { return storage_[y * width_ + x].value; }

  /// @brief Reading access to the cell at the given coordinates.
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @return Const reference to the cell at the given coordinates.
  [[nodiscard]] const auto& cell(int x, int y) const { return storage_[y * width_ + x].value; }

  /// @brief Returns the virtual size of the map (number of cells).
  [[nodiscard]] auto size() const { return storage_.size(); }

  /// @brief Returns the width of the map (number of cells).
  [[nodiscard]] auto width() const { return width_; }

  /// @brief Returns the height of the map (number of cells).
  [[nodiscard]] auto height() const { return height_; }

 private:
  // Crappy workaround to avoid special cases for bool
  struct WrappedT {
    T value{};
  };

  std::size_t width_;
  std::size_t height_;
  std::vector<WrappedT> storage_;
};

}  // namespace beluga

#endif

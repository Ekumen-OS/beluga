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

#ifndef BELUGA_SENSOR_DATA_VALUE_GRID_HPP
#define BELUGA_SENSOR_DATA_VALUE_GRID_HPP

#include <cstdint>
#include <utility>
#include <vector>

#include <beluga/sensor/data/linear_grid.hpp>

/**
 * \file
 * \brief Implementation of generic value grid.
 */

namespace beluga {

/// Generic 2D linear value grid.
/**
 * \tparam T Any copyable type.
 */
template <typename T>
class ValueGrid2 : public BaseLinearGrid2<ValueGrid2<T>> {
 public:
  /// Constructs the grid.
  /**
   * \param data Grid data.
   * \param width Grid width. Must evenly divide `data` size.
   * \param resolution Grid resolution.
   */
  explicit ValueGrid2(std::vector<T> data, std::size_t width, double resolution = 1.)
      : data_(std::move(data)), width_(width), height_(data_.size() / width), resolution_(resolution) {
    assert(data_.size() % width == 0);
  }

  /// Gets grid width.
  [[nodiscard]] std::size_t width() const { return width_; }

  /// Gets grid height.
  [[nodiscard]] std::size_t height() const { return height_; }

  /// Gets grid resolution.
  [[nodiscard]] double resolution() const { return resolution_; }

  /// Gets grid size (ie. number of grid cells).
  [[nodiscard]] std::size_t size() const { return data_.size(); }

  /// Gets grid data.
  [[nodiscard]] const std::vector<T>& data() const { return data_; }

 private:
  std::vector<T> data_;
  std::size_t width_;
  std::size_t height_;
  double resolution_;
};

}  // namespace beluga

#endif

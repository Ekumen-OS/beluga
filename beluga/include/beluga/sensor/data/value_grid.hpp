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

#include <beluga/sensor/data/dense_grid.hpp>
#include <beluga/sensor/data/regular_grid.hpp>
#include <beluga/sensor/data/value_grid.hpp>

#include <range/v3/view/common.hpp>

#include <ciabatta/ciabatta.hpp>

/**
 * \file
 * \brief Implementation of generic value grid.
 */

namespace beluga {

/// Generic 2D linear value grid.
/**
 * \tparam Mixin Base mixin class.
 * \tparam T Any copyable type.
 */
template <typename Mixin, typename T>
class ValueGrid2Mixin : public Mixin {
 public:
  /// Constructs the grid.
  /**
   * \param data Grid data.
   * \param width Grid width. Must evenly divide `data` size.
   * \param resolution Grid resolution.
   * \param args Arguments to be forwarded to the mixin components.
   */
  template <typename... Args>
  explicit ValueGrid2Mixin(std::vector<T> data, std::size_t width, double resolution, Args&&... args)
      : Mixin(std::forward<Args>(args)...),
        data_(std::move(data)),
        width_(width),
        height_(data_.size() / width),
        resolution_(resolution) {
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

  /// Return a range containing the grid data in row-major order.
  [[nodiscard]] const auto row_major_scan() const { return data_ | ranges::views::common; }

  using Mixin::data_at;

  /// Gets cell data, if included.
  /**
   * \param xi Grid cell x-axis coordinate.
   * \param yi Grid cell y-axis coordinate.
   * \return Cell data if included, `std::nullopt` otherwise.
   */
  [[nodiscard]] auto data_at(int xi, int yi) const {
    const auto index_at = [this](int xi, int yi) {
      return static_cast<std::size_t>(yi) * this->self().width() + static_cast<std::size_t>(xi);
    };
    return this->self().contains(xi, yi) ? std::make_optional(this->self().data()[index_at(xi, yi)]) : std::nullopt;
  }

 private:
  std::vector<T> data_;
  std::size_t width_;
  std::size_t height_;
  double resolution_;
};

/// Generic 2D linear value grid.
/**
 * \tparam ValueType Type used for grid cell value representation.
 */
template <class ValueType>
using ValueGrid2 = ciabatta::
    mixin<ciabatta::curry<ValueGrid2Mixin, ValueType>::template mixin, BaseDenseGrid2Mixin, BaseRegularGrid2Mixin>;

}  // namespace beluga

#endif

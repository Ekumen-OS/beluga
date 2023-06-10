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

#ifndef BELUGA_TEST_STATIC_OCCUPANCY_GRID_HPP
#define BELUGA_TEST_STATIC_OCCUPANCY_GRID_HPP

#include <array>
#include <cstdint>
#include <initializer_list>
#include <vector>

#include <beluga/sensor/data/dense_grid.hpp>
#include <beluga/sensor/data/regular_grid.hpp>

#include <ciabatta/ciabatta.hpp>

#include "beluga/sensor/data/occupancy_grid.hpp"

#include <sophus/se2.hpp>

namespace beluga::testing {

template <std::size_t Rows, std::size_t Cols>
class PlainGridStorage {
 public:
  using StorageType = std::array<bool, Rows * Cols>;

  /// @brief Default constructor. Constructs a map with all cells marked as empty.
  PlainGridStorage() : grid_storage_(std::make_unique<StorageType>()) { grid_storage_->fill(false); }

  /// @brief Constructs a map with the given initial values.
  /// @param init_values Initial contents of the map, in row-major order.
  PlainGridStorage(std::initializer_list<bool> array) : grid_storage_(std::make_unique<StorageType>()) {
    const auto n = std::min(array.size(), grid_storage_->size());
    std::copy_n(array.begin(), n, grid_storage_->begin());
    if (n < grid_storage_->size()) {
      std::fill(grid_storage_->begin() + n, grid_storage_->end(), false);
    }
  }

  /// @brief  Returns a reference to the underlying data. Too low level, will be removed in the future.
  /// @return A reference to the underlying data.
  [[nodiscard]] auto& data() { return *grid_storage_; }

  /// @brief  Returns a const reference to the underlying data. Too low level, will be removed in the future.
  /// @return A const reference to the underlying data.
  [[nodiscard]] const auto& data() const { return *grid_storage_; }

  /// @brief Access to the cell at the given coordinates.
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @return Reference to the cell at the given coordinates.
  [[nodiscard]] auto& cell(int x, int y) { return (*grid_storage_)[y * Cols + x]; }

  /// @brief Reading access to the cell at the given coordinates.
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @return Const reference to the cell at the given coordinates.
  [[nodiscard]] const auto& cell(int x, int y) const { return (*grid_storage_)[y * Cols + x]; }

  /// @brief Returns the virtual size of the map (number of cells).
  [[nodiscard]] auto size() const { return Rows * Cols; }

  /// @brief Returns the width of the map (number of cells).
  [[nodiscard]] auto width() const { return Cols; }

  /// @brief Returns the height of the map (number of cells).
  [[nodiscard]] auto height() const { return Rows; }

  // Somewhere in the code something needs the copy constructor to be exist, probably something to fix.
  PlainGridStorage(const PlainGridStorage& src) { grid_storage_ = std::make_unique<StorageType>(*src.grid_storage_); }
  PlainGridStorage& operator=(const PlainGridStorage& src) = delete;

  PlainGridStorage(PlainGridStorage&&) = default;
  PlainGridStorage& operator=(PlainGridStorage&&) = default;

 private:
  std::unique_ptr<StorageType> grid_storage_;
};

template <std::size_t Rows, std::size_t Cols>
struct GridSize {
  static constexpr std::size_t rows = Rows;
  static constexpr std::size_t cols = Cols;
};

template <class Mixin, class GridSize>
class StaticOccupancyGridMixin : public Mixin {
 public:
  struct ValueTraits {
    [[nodiscard]] bool is_free(bool value) const { return !value; }
    [[nodiscard]] bool is_unknown(bool) const { return false; }
    [[nodiscard]] bool is_occupied(bool value) const { return value; }
  };

  template <typename... Args>
  explicit StaticOccupancyGridMixin(
      PlainGridStorage<GridSize::rows, GridSize::cols>&& grid_storage,
      double resolution,
      const Sophus::SE2d& origin,
      Args&&... args)
      : Mixin(std::forward<Args>(args)...),
        grid_storage_{std::move(grid_storage)},
        origin_(origin),
        resolution_{resolution} {}

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] auto& data() { return grid_storage_.data(); }
  [[nodiscard]] const auto& data() const { return grid_storage_.data(); }
  [[nodiscard]] std::size_t size() const { return grid_storage_.size(); }

  [[nodiscard]] std::size_t width() const { return GridSize::cols; }
  [[nodiscard]] std::size_t height() const { return GridSize::rows; }
  [[nodiscard]] double resolution() const { return resolution_; }

  [[nodiscard]] auto value_traits() const { return ValueTraits{}; }

 private:
  PlainGridStorage<GridSize::rows, GridSize::cols> grid_storage_;
  Sophus::SE2d origin_;
  double resolution_;
};

template <std::size_t Rows, std::size_t Cols>
using StaticOccupancyGrid = ciabatta::mixin<
    ciabatta::curry<StaticOccupancyGridMixin, GridSize<Rows, Cols>>::template mixin,
    BaseOccupancyGrid2Mixin,
    BaseDenseGrid2Mixin,
    BaseRegularGrid2Mixin>;

}  // namespace beluga::testing

#endif

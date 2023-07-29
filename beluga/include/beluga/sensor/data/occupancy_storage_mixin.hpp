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

#ifndef BELUGA_SENSOR_DATA_OCCUPANCY_STORAGE_MIXIN_HPP
#define BELUGA_SENSOR_DATA_OCCUPANCY_STORAGE_MIXIN_HPP

#include <cstdint>

#include <ciabatta/ciabatta.hpp>
#include <sophus/se2.hpp>

/*
 * \file
 * \brief Concepts and abstract implementations of occupancy grids.
 */

namespace beluga {

/// Occupancy grid storage interface layer.
/**
 * \tparam Mixin Mixin type
 * \tparam ValueType Occupancy grid cell value type
 * \tparam ValueTraitsType Occupancy grid cell value traits type
 * \tparam MapStorageType Occupancy grid storage type
 */
template <class Mixin, typename ValueType, typename ValueTraitsType, typename MapStorageType>
class OccupancyStorageMixin : public Mixin {
 public:
  /// @brief Type of the storage used by the occupancy grid.
  using MapStorage = MapStorageType;
  /// @brief Traits that determine empty and occupied values for the occupancy grid.
  using ValueTraits = ValueTraitsType;

  /// @brief Mixin constructor
  /// @param grid_storage Grid instance preloaded with the occupancy data
  /// @param resolution Occupancy grid resolution value.
  /// @param origin Occupancy grid origin pose.
  /// @param ...args arguments to be forwarded to other mixins in the chain
  template <typename... Args>
  explicit OccupancyStorageMixin(
      MapStorage&& grid_storage,
      double resolution,
      const Sophus::SE2d& origin,
      Args&&... args)
      : Mixin(std::forward<Args>(args)...),
        grid_storage_{std::move(grid_storage)},
        origin_(origin),
        resolution_{resolution} {}

  /// @brief Returns the origin pose of the occupancy grid.
  /// @return Origin pose.
  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  /// @brief Returns the width of the occupancy grid.
  /// @return Width of the grid.
  [[nodiscard]] std::size_t width() const { return grid_storage_.width(); }

  /// @brief Returns the height of the occupancy grid.
  /// @return Height of the grid.
  [[nodiscard]] std::size_t height() const { return grid_storage_.height(); }

  /// @brief Returns the resolution of the occupancy grid.
  /// @return Resolution of the grid.
  [[nodiscard]] double resolution() const { return resolution_; }

  /// @brief Returns the value traits of the occupancy grid.
  /// @return Value traits of the grid.
  [[nodiscard]] auto value_traits() const { return ValueTraits{}; }

  /// @brief Returns the occupancy grid cell at the given coordinates.
  /// @param x x-axis coordinate.
  /// @param y y-axis coordinate.
  /// @return Occupancy grid cell.
  [[nodiscard]] const auto& cell(const int x, const int y) const { return grid_storage_.cell(x, y); }

  /// @brief Returns the occupancy grid cell at the given coordinates.
  /// @param x x-axis coordinate.
  /// @param y y-axis coordinate.
  /// @return Occupancy grid cell.
  [[nodiscard]] auto& cell(const int x, const int y) { return grid_storage_.cell(x, y); }

 private:
  MapStorage grid_storage_;
  Sophus::SE2d origin_;
  double resolution_;
};

}  // namespace beluga

#endif

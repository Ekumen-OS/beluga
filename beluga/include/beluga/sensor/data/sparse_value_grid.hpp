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

#ifndef BELUGA_SENSOR_DATA_SPARSE_VALUE_GRID_HPP
#define BELUGA_SENSOR_DATA_SPARSE_VALUE_GRID_HPP

#include <Eigen/Core>
#include <type_traits>

#include <beluga/sensor/data/regular_grid.hpp>
/**
 * \file
 * \brief Implementation of generic dense value grid.
 */

namespace beluga {

/// Generic 2D sparse value grid.
/**
 * \tparam T Any copyable type.
 * \tparam map_type TODO: complete docstring
 */
template <typename MapType>
class SparseValueGrid : public BaseRegularGrid2<SparseValueGrid<MapType>> {
 public:
  SparseValueGrid() = default;
  using map_type = MapType;
  using mapped_type = typename map_type::mapped_type;
  using key_type = typename map_type::key_type;
  static_assert(std::is_same_v<key_type, Eigen::Vector2i>);
  /// Constructs the grid.
  /**
   * \param data Grid data.
   * \param resolution Grid resolution.
   */
  explicit SparseValueGrid(map_type data, double resolution = 1.) : data_(std::move(data)), resolution_(resolution) {
    assert(resolution_ > 0);
  }

  /// Gets grid resolution.
  [[nodiscard]] double resolution() const { return resolution_; }

  /// Gets grid size (ie. number of grid cells).
  [[nodiscard]] std::size_t size() const { return data_.size(); }

  /// Gets grid data.
  [[nodiscard]] const map_type& data() const { return data_; }

  /// Gets grid data at cell_index.
  [[nodiscard]] const mapped_type& data_at_index(const Eigen::Vector2i& cell_index) const {
    return data_.at(cell_index);
  }

  /// Gets grid data at {cell_corodinates}.
  [[nodiscard]] const mapped_type& data_at(const Eigen::Vector2d& cell_coordinates) const {
    return data_.at(this->self().cell_near(cell_coordinates));
  }

  /// Gets grid data at {x, y}.
  [[nodiscard]] const mapped_type& data_at(const double x, const double y) const {
    return data_.at(this->self().cell_near(x, y));
  }

  /// Returns whether the cell index {x, y} exists in the grid or not.
  [[nodiscard]] bool contains_index(const int x, const int y) const { return contains_index(Eigen::Vector2i{x, y}); }

  /// Returns whether 'cell_index' exists in the grid or not.
  [[nodiscard]] bool contains_index(const Eigen::Vector2i& cell_index) const {
    return data_.find(cell_index) != data_.end();
  }

 private:
  const map_type data_;
  const double resolution_ = 1.0;
};

}  // namespace beluga

#endif

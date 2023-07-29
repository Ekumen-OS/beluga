// Copyright 2022-2023 Ekumen, Inc.
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

#ifndef BELUGA_AMCL_ROS_OCCUPANCY_GRID_HPP
#define BELUGA_AMCL_ROS_OCCUPANCY_GRID_HPP

#include <beluga/sensor/data/dense_grid2_mixin.hpp>
#include <beluga/sensor/data/linear_grid_storage.hpp>
#include <beluga/sensor/data/occupancy_grid2_mixin.hpp>
#include <beluga/sensor/data/occupancy_storage_mixin.hpp>
#include <beluga/sensor/data/regular_grid2_mixin.hpp>
#include <beluga/sensor/data/value_grid2_mixin.hpp>
#include <beluga_amcl/ros_interfaces.hpp>

namespace beluga_amcl {

namespace ros_occupancy_grid_types {
struct ROSMapValueTraits {
  // https://wiki.ros.org/map_server#Value_Interpretation
  static constexpr std::int8_t kFreeValue = 0;
  static constexpr std::int8_t kUnknownValue = -1;
  static constexpr std::int8_t kOccupiedValue = 100;

  [[nodiscard]] static constexpr bool is_free(std::int8_t value) { return value == kFreeValue; }

  [[nodiscard]] static constexpr bool is_unknown(std::int8_t value) { return value == kUnknownValue; }

  [[nodiscard]] static constexpr bool is_occupied(std::int8_t value) { return value == kOccupiedValue; }
};

using ROSMapCellType = std::int8_t;

}  // namespace ros_occupancy_grid_types

using ROSOccupancyGrid = ciabatta::mixin<
    ciabatta::curry<
        beluga::OccupancyStorageMixin,
        ros_occupancy_grid_types::ROSMapCellType,
        ros_occupancy_grid_types::ROSMapValueTraits,
        beluga::LinearGridStorage<ros_occupancy_grid_types::ROSMapCellType>>::template mixin,
    beluga::OccupancyGrid2Mixin,
    beluga::DenseGrid2Mixin,
    beluga::RegularGrid2Mixin>;

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL_ROS_OCCUPANCY_GRID_HPP

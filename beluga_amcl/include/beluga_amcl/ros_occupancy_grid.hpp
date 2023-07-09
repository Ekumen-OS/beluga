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

#ifndef BELUGA_AMCL__ROS_OCCUPANCY_GRID_HPP_
#define BELUGA_AMCL__ROS_OCCUPANCY_GRID_HPP_

#include <beluga_amcl/ros_interfaces.hpp>
#include <beluga/sensor/data/dense_grid.hpp>
#include <beluga/sensor/data/linear_grid_storage.hpp>
#include <beluga/sensor/data/occupancy_grid.hpp>
#include <beluga/sensor/data/occupancy_grid_storage.hpp>
#include <beluga/sensor/data/regular_grid.hpp>
#include <beluga/sensor/data/value_grid.hpp>


namespace beluga_amcl
{

namespace ros_occupancy_grid_types
{
struct ROSMapValueTraits
{
  // https://wiki.ros.org/map_server#Value_Interpretation
  static constexpr std::int8_t free_value = 0;
  static constexpr std::int8_t unknown_value = -1;
  static constexpr std::int8_t occupied_value = 100;

  bool is_free(std::int8_t value) const
  {
    return value == free_value;
  }

  bool is_unknown(std::int8_t value) const
  {
    return value == unknown_value;
  }

  bool is_occupied(std::int8_t value) const
  {
    return value == occupied_value;
  }
};

using ROSMapCellType = std::int8_t;

}  // namespace ros_occupancy_grid_types

using ROSOccupancyGrid = ciabatta::mixin<
  ciabatta::curry<beluga::OccupancyStorageMixin,
  ros_occupancy_grid_types::ROSMapCellType,
  ros_occupancy_grid_types::ROSMapValueTraits,
  beluga::LinearGridStorage<ros_occupancy_grid_types::ROSMapCellType>>::template mixin,
  beluga::BaseOccupancyGrid2Mixin,
  beluga::BaseDenseGrid2Mixin,
  beluga::BaseRegularGrid2Mixin>;


}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__ROS_OCCUPANCY_GRID_HPP_

// Copyright 2024 Ekumen, Inc.
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

#ifndef BELUGA_ROS_LIKELIHOOD_FIELD_HPP
#define BELUGA_ROS_LIKELIHOOD_FIELD_HPP

#include <cstdint>

#include <beluga/sensor/data/value_grid.hpp>
#include <beluga_ros/tf2_sophus.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

/**
 * \file
 * \brief Utilities for likelihood data over ROS interfaces.
 */

namespace beluga_ros {

/// Convert from likelihood data to ROS2 message.
template <typename T>
inline void assign_likelihood_field(
    const beluga::ValueGrid2<T>& likelihood_field,
    const Sophus::SE2d& origin,
    nav_msgs::msg::OccupancyGrid& message) {
  // Set metadata
  message.info.width = static_cast<unsigned int>(likelihood_field.width());
  message.info.height = static_cast<unsigned int>(likelihood_field.height());
  message.info.resolution = static_cast<float>(likelihood_field.resolution());
  tf2::toMsg(origin, message.info.origin);  // origin -> Pose: [x,y,z],[w,x,y,z]

  // Populate the data field with the grid data
  const auto& grid_data = likelihood_field.data();
  message.data.resize(likelihood_field.size());

  // Find min and max values for normalization
  const auto [min_it, max_it] = std::minmax_element(grid_data.begin(), grid_data.end());
  const float min_val = *min_it;
  const float max_val = *max_it;
  const float range = max_val - min_val;

  // Handle degenerate case (flat grid) by filling the occupancy grid with zeros
  if (range <= std::numeric_limits<float>::epsilon()) {
    // All values are the same; treat as unknown or flat
    std::fill(message.data.begin(), message.data.end(), 0);
    return;
  }

  // Normalizing each cell to [0, 100] - To be consistent with nav2:
  // navigation2/nav2_costmap_2d/src/costmap_2d_publisher.cpp
  for (std::size_t i = 0; i < grid_data.size(); ++i) {
    const float normalized = (grid_data[i] - min_val) / range;
    message.data[i] = static_cast<int8_t>(normalized * 100.0f);  // Scale to [0, 100]
  }
}

}  // namespace beluga_ros

#endif  // BELUGA_ROS_LIKELIHOOD_FIELD_HPP

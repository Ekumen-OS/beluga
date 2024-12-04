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

#ifndef BELUGA_ROS_NDT_ELLIPSOID_HPP
#define BELUGA_ROS_NDT_ELLIPSOID_HPP

#include <beluga_ros/messages.hpp>


/**
 * \file
 * \brief Utilities for NDT I/O over ROS interfaces.
 */

namespace beluga_ros {

/// 
template <typename MapType, int NDim>
beluga_ros::msg::MarkerArray assign_obstacle_map(const beluga::SparseValueGrid<MapType, NDim>& grid){

  // Get data from the grid
  auto& map = grid.data();

  beluga_ros::msg::MarkerArray message{};
  {
    beluga_ros::msg::Marker marker;
    marker.ns = "obstacles";
    marker.action = beluga_ros::msg::Marker::DELETE;
  }

  int idCount = 0;
  for (const auto& [key, cell] : map)
  {
    beluga_ros::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.id = idCount++;
    marker.ns = "obstacles";
    marker.type = beluga_ros::msg::Marker::SPHERE;
    marker.action = beluga_ros::msg::Marker::ADD;

    // Set the position
    marker.pose.position.x = cell.mean[0];
    marker.pose.position.y = cell.mean[1];
    marker.pose.position.z = cell.mean[2];

    // Set the scale based on the covariance
    marker.scale.x = 0.5f;
    marker.scale.y = 0.5f;
    marker.scale.z = 0.5f;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    // Add the marker
    message.markers.push_back(marker);
  }

  return message;
}

}  // namespace beluga_ros

#endif  // BELUGA_ROS_NDT_ELLIPSOID_HPP

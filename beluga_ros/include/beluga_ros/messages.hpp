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

#ifndef BELUGA_ROS_MESSAGES_HPP
#define BELUGA_ROS_MESSAGES_HPP

#if BELUGA_ROS_VERSION == 2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <rclcpp/time.hpp>
#elif BELUGA_ROS_VERSION == 1
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include <ros/time.h>
#else
#error BELUGA_ROS_VERSION is not defined or invalid
#endif

namespace beluga_ros {

/// Compatibility layer for ROS 1 and ROS 2 messages.
namespace msg {

#if BELUGA_ROS_VERSION == 2

using LaserScan = sensor_msgs::msg::LaserScan;
using LaserScanConstSharedPtr = LaserScan::ConstSharedPtr;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using OccupancyGridConstSharedPtr = OccupancyGrid::ConstSharedPtr;
using Pose = geometry_msgs::msg::Pose;
using PoseArray = geometry_msgs::msg::PoseArray;
using Transform = geometry_msgs::msg::Transform;

#elif BELUGA_ROS_VERSION == 1

using LaserScan = sensor_msgs::LaserScan;
using LaserScanConstSharedPtr = LaserScan::ConstPtr;
using OccupancyGrid = nav_msgs::OccupancyGrid;
using OccupancyGridConstSharedPtr = OccupancyGrid::ConstPtr;
using Pose = geometry_msgs::Pose;
using PoseArray = geometry_msgs::PoseArray;
using Transform = geometry_msgs::Transform;

#else
#error BELUGA_ROS_VERSION is not defined or invalid
#endif

}  // namespace msg

namespace detail {

#if BELUGA_ROS_VERSION == 2

using Time = rclcpp::Time;

#elif BELUGA_ROS_VERSION == 1

using Time = ros::Time;

#else
#error BELUGA_ROS_VERSION is not defined or invalid
#endif

}  // namespace detail

template <class Message>
Message& stamp_message(std::string_view frame_id, detail::Time timestamp, Message& message) {
  message.header.frame_id = frame_id;
  message.header.stamp = timestamp;
  return message;
}

}  // namespace beluga_ros

#endif  // BELUGA_ROS_MESSAGES_HPP

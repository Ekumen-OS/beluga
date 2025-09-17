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

#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/time.hpp>

namespace beluga_ros {

/// Stamp a message with a frame ID and timestamp.
/**
 * \param frame_id Frame ID to stamp the message with.
 * \param timestamp Time to stamp the message at.
 * \param[out] message Message to be stamped.
 * \tparam Message A message type with a header.
 */
template <class Message>
Message& stamp_message(std::string_view frame_id, rclcpp::Time timestamp, Message& message) {
  message.header.frame_id = frame_id;
  message.header.stamp = timestamp;
  return message;
}

/// Stamp all markers in a marker array message with a frame ID and timestamp.
/**
 * \param frame_id Frame ID to stamp markers with.
 * \param timestamp Time to stamp markers at.
 * \param[out] message Message to be stamped.
 */
inline visualization_msgs::msg::MarkerArray&
stamp_message(std::string_view frame_id, rclcpp::Time timestamp, visualization_msgs::msg::MarkerArray& message) {
  for (auto& marker : message.markers) {
    stamp_message(frame_id, timestamp, marker);
  }
  return message;
}

}  // namespace beluga_ros

#endif  // BELUGA_ROS_MESSAGES_HPP

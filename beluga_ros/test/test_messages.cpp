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

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "beluga_ros/messages.hpp"

namespace {

TEST(TestMessages, Stamping) {
  auto message = geometry_msgs::msg::PoseArray{};
  beluga_ros::stamp_message("some_frame", {5, 0}, message);
  EXPECT_EQ(message.header.frame_id, "some_frame");
  EXPECT_EQ(message.header.stamp.sec, 5);
}

TEST(TestMessages, StampingMany) {
  auto message = visualization_msgs::msg::MarkerArray{};
  message.markers.insert(message.markers.end(), 5U, visualization_msgs::msg::Marker{});
  beluga_ros::stamp_message("some_frame", {5, 0}, message);
  for (const auto& marker : message.markers) {
    EXPECT_EQ(marker.header.frame_id, "some_frame");
    EXPECT_EQ(marker.header.stamp.sec, 5);
  }
}

}  // namespace

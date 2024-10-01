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

#include "beluga_ros/messages.hpp"

namespace {

TEST(TestMessages, Instantiation) {
  [[maybe_unused]] auto msg1 = beluga_ros::msg::ColorRGBA{};
  [[maybe_unused]] auto msg2 = beluga_ros::msg::LaserScan{};
  [[maybe_unused]] auto msg3 = beluga_ros::msg::Marker{};
  [[maybe_unused]] auto msg4 = beluga_ros::msg::MarkerArray{};
  [[maybe_unused]] auto msg5 = beluga_ros::msg::OccupancyGrid{};
  [[maybe_unused]] auto msg6 = beluga_ros::msg::Point{};
  [[maybe_unused]] auto msg7 = beluga_ros::msg::Pose{};
  [[maybe_unused]] auto msg8 = beluga_ros::msg::PoseArray{};
  [[maybe_unused]] auto msg9 = beluga_ros::msg::Transform{};
  [[maybe_unused]] auto msg10 = beluga_ros::msg::PointCloud2{};
}

TEST(TestMessages, Stamping) {
  auto message = beluga_ros::msg::PoseArray{};
  beluga_ros::stamp_message("some_frame", {5, 0}, message);
  EXPECT_EQ(message.header.frame_id, "some_frame");
  EXPECT_EQ(message.header.stamp.sec, 5);
}

TEST(TestMessages, StampingMany) {
  auto message = beluga_ros::msg::MarkerArray{};
  message.markers.insert(message.markers.end(), 5U, beluga_ros::msg::Marker{});
  beluga_ros::stamp_message("some_frame", {5, 0}, message);
  for (const auto& marker : message.markers) {
    EXPECT_EQ(marker.header.frame_id, "some_frame");
    EXPECT_EQ(marker.header.stamp.sec, 5);
  }
}

}  // namespace

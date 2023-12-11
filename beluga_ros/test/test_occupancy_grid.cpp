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
#include <beluga_ros/occupancy_grid.hpp>

#if BELUGA_ROS_VERSION == 1
#include <boost/smart_ptr.hpp>
#endif

namespace {

TEST(TestOccupancyGrid, Instantiation) {
  constexpr std::size_t kWidth = 100;
  constexpr std::size_t kHeight = 200;

#if BELUGA_ROS_VERSION == 2
  auto message = std::make_shared<beluga_ros::msg::OccupancyGrid>();
#elif BELUGA_ROS_VERSION == 1
  auto message = boost::make_shared<beluga_ros::msg::OccupancyGrid>();
#else
#error BELUGA_ROS_VERSION is not defined or invalid
#endif
  message->info.resolution = 0.1F;
  message->info.width = kWidth;
  message->info.height = kHeight;
  message->info.origin.position.x = 1;
  message->info.origin.position.y = 2;
  message->info.origin.position.z = 0;
  message->data = std::vector<std::int8_t>(kWidth * kHeight);

  auto grid = beluga_ros::OccupancyGrid{message};
  ASSERT_EQ(grid.resolution(), message->info.resolution);
  ASSERT_EQ(grid.size(), message->data.size());
  ASSERT_EQ(grid.width(), kWidth);
  ASSERT_EQ(grid.height(), kHeight);
}

TEST(TestOccupancyGrid, ValueTraitsIsFree) {
  ASSERT_TRUE(beluga_ros::OccupancyGrid::ValueTraits::is_free(0));
  ASSERT_FALSE(beluga_ros::OccupancyGrid::ValueTraits::is_free(50));
  ASSERT_FALSE(beluga_ros::OccupancyGrid::ValueTraits::is_free(100));
  ASSERT_FALSE(beluga_ros::OccupancyGrid::ValueTraits::is_free(-1));
}

TEST(TestOccupancyGrid, ValueTraitsIsUnkown) {
  ASSERT_FALSE(beluga_ros::OccupancyGrid::ValueTraits::is_unknown(0));
  ASSERT_FALSE(beluga_ros::OccupancyGrid::ValueTraits::is_unknown(50));
  ASSERT_FALSE(beluga_ros::OccupancyGrid::ValueTraits::is_unknown(100));
  ASSERT_TRUE(beluga_ros::OccupancyGrid::ValueTraits::is_unknown(-1));
}

TEST(TestOccupancyGrid, ValueTraitsIsOccupied) {
  ASSERT_FALSE(beluga_ros::OccupancyGrid::ValueTraits::is_occupied(0));
  ASSERT_FALSE(beluga_ros::OccupancyGrid::ValueTraits::is_occupied(50));
  ASSERT_TRUE(beluga_ros::OccupancyGrid::ValueTraits::is_occupied(100));
  ASSERT_FALSE(beluga_ros::OccupancyGrid::ValueTraits::is_occupied(-1));
}

}  // namespace

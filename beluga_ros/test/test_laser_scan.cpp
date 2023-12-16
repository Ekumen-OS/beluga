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

#include <gmock/gmock.h>
#include <beluga_ros/laser_scan.hpp>

#include <range/v3/range/conversion.hpp>

#if BELUGA_ROS_VERSION == 1
#include <boost/smart_ptr.hpp>
#endif

namespace {

auto make_message() {
#if BELUGA_ROS_VERSION == 2
  return std::make_shared<beluga_ros::msg::LaserScan>();
#elif BELUGA_ROS_VERSION == 1
  return boost::make_shared<beluga_ros::msg::LaserScan>();
#endif
}

TEST(TestLaserScan, MinMaxRangeFromMessage) {
  auto message = make_message();
  message->ranges = std::vector<float>{1., 2., 3.};
  message->range_min = 10.f;
  message->range_max = 100.f;
  auto scan = beluga_ros::LaserScan(message);
  ASSERT_EQ(scan.min_range(), 10.);
  ASSERT_EQ(scan.max_range(), 100.);
}

TEST(TestLaserScan, MinMaxRangeFromConstructor) {
  auto message = make_message();
  message->range_min = 10.f;
  message->range_max = 100.f;
  const auto origin = Sophus::SE3d{};
  constexpr auto kMaxBeams = 100UL;
  constexpr auto kMinRange = 15.;
  constexpr auto kMaxRange = 95.;
  auto scan = beluga_ros::LaserScan(message, origin, kMaxBeams, kMinRange, kMaxRange);
  ASSERT_EQ(scan.min_range(), 15.);
  ASSERT_EQ(scan.max_range(), 95.);
}

TEST(TestLaserScan, LimitMaxBeams) {
  auto message = make_message();
  message->ranges = std::vector<float>{1., 2., 3.};
  const auto origin = Sophus::SE3d{};
  constexpr auto kMaxBeams = 2UL;
  auto scan = beluga_ros::LaserScan(message, origin, kMaxBeams);
  auto ranges = scan.ranges() | ranges::to<std::vector>;
  ASSERT_EQ(ranges.size(), 2UL);
}

TEST(TestLaserScan, AngleIncrements) {
  auto message = make_message();
  message->ranges = std::vector<float>{1., 2., 3.};
  message->range_min = 0.f;
  message->range_max = 100.f;
  message->angle_min = 0.f;
  message->angle_max = 3.14f;
  message->angle_increment = 0.1f;
  const auto origin = Sophus::SE3d{};
  constexpr auto kMaxBeams = 100UL;
  auto scan = beluga_ros::LaserScan(message, origin, kMaxBeams);
  auto angles = scan.angles() | ranges::to<std::vector>;
  ASSERT_NEAR(angles[0], 0.0, 0.001);
  ASSERT_NEAR(angles[1], 0.1, 0.001);
  ASSERT_NEAR(angles[2], 0.2, 0.001);
}

}  // namespace

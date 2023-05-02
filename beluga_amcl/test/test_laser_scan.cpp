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

#include <beluga_amcl/laser_scan.hpp>

namespace {

TEST(LaserScan, NoBeams)
{
  auto scan_message = std::make_shared<sensor_msgs::msg::LaserScan>();
  auto laser_scan = beluga_amcl::LaserScan{std::move(scan_message)};
  ASSERT_EQ(laser_scan.size(), 0UL);
}

TEST(LaserScan, MaxBeamsZero)
{
  auto scan_message = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_message->ranges = std::vector<float>{5., 6., 7., 8., 9., 10.};
  auto laser_scan = beluga_amcl::LaserScan{std::move(scan_message), 0};
  ASSERT_EQ(laser_scan.size(), 0UL);
}

TEST(LaserScan, TakeThreeBeamsFromSix)
{
  auto scan_message = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_message->ranges = std::vector<float>{5., 6., 7., 8., 9., 10.};
  auto laser_scan = beluga_amcl::LaserScan{std::move(scan_message), 3UL};
  ASSERT_EQ(laser_scan.size(), 3UL);
  auto ranges = laser_scan.ranges() | ranges::to<std::vector>;
  EXPECT_DOUBLE_EQ(ranges[0], 5.0);
  EXPECT_DOUBLE_EQ(ranges[1], 7.0);
  EXPECT_DOUBLE_EQ(ranges[2], 9.0);
}

TEST(LaserScan, TakeThreeBeamsFromNine)
{
  auto scan_message = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_message->ranges = std::vector<float>{5., 6., 7., 8., 9., 10., 11., 12., 13.};
  auto laser_scan = beluga_amcl::LaserScan{std::move(scan_message), 3UL};
  ASSERT_EQ(laser_scan.size(), 3UL);
  auto ranges = laser_scan.ranges() | ranges::to<std::vector>;
  EXPECT_DOUBLE_EQ(ranges[0], 5.0);
  EXPECT_DOUBLE_EQ(ranges[1], 9.0);
  EXPECT_DOUBLE_EQ(ranges[2], 13.0);
}

TEST(LaserScan, TransformIdentity)
{
  auto scan_message = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_message->range_min = 0.0;
  scan_message->range_max = 20.0;
  scan_message->angle_min = Sophus::Constants<float>::pi() / 4;
  scan_message->angle_max = Sophus::Constants<float>::pi() * 3 / 4;
  scan_message->angle_increment = Sophus::Constants<float>::pi() / 4;
  scan_message->time_increment = 0;
  scan_message->ranges = std::vector<float>{5., 5., 5.};
  auto laser_scan = beluga_amcl::LaserScan{std::move(scan_message), 3UL};
  ASSERT_EQ(laser_scan.size(), 3UL);

  auto points = laser_scan.points_in_cartesian_coordinates() | ranges::to<std::vector>;
  EXPECT_NEAR(points[0].x(), 3.535, 0.001);
  EXPECT_NEAR(points[0].y(), 3.535, 0.001);
  EXPECT_NEAR(points[1].x(), 0.000, 0.001);
  EXPECT_NEAR(points[1].y(), 5.000, 0.001);
  EXPECT_NEAR(points[2].x(), -3.535, 0.001);
  EXPECT_NEAR(points[2].y(), 3.535, 0.001);
}

// TEST(LaserScan, Transform)
// {
//   auto scan_message = std::make_shared<sensor_msgs::msg::LaserScan>();
//   scan_message->range_min = 0.0;
//   scan_message->range_max = 20.0;
//   scan_message->angle_min = Sophus::Constants<float>::pi() / 4.;
//   scan_message->angle_max = Sophus::Constants<float>::pi() * 3. / 4.;
//   scan_message->angle_increment = Sophus::Constants<float>::pi() / 4.;
//   scan_message->time_increment = 0;
//   scan_message->ranges = std::vector<float>{5., 5., 5.};
//   const auto output = beluga_amcl::utils::make_points_from_laser_scan(
//     laser_scan,
//     Sophus::SE3d{
//       Sophus::SO3d{
//         Eigen::AngleAxisd(-Sophus::Constants<float>::pi() * 3 / 4, Eigen::Vector3d::UnitX()) *
//         Eigen::AngleAxisd(Sophus::Constants<float>::pi() / 4, Eigen::Vector3d::UnitY())},
//       Eigen::Vector3d{1.0, 2.0, 3.0}},
//     laser_scan.ranges.size(),
//     laser_scan.range_min,
//     laser_scan.range_max
//   );
//   ASSERT_EQ(output.size(), 3UL);
//   EXPECT_NEAR(output[0].first, 3.500, 0.001);
//   EXPECT_NEAR(output[0].second, -2.267, 0.001);
//   EXPECT_NEAR(output[1].first, 1.000, 0.001);
//   EXPECT_NEAR(output[1].second, -1.535, 0.001);
//   EXPECT_NEAR(output[2].first, -1.500, 0.001);
//   EXPECT_NEAR(output[2].second, 1.267, 0.001);
// }

}  // namespace

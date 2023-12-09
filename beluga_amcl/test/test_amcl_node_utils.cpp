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

#include <sstream>

#include <beluga_amcl/amcl_node_utils.hpp>

namespace {

/// Returns the norm of a 2D point represented by a pair of doubles.
double norm(const std::pair<double, double>& point) {
  return std::sqrt(point.first * point.first + point.second * point.second);
}

TEST(MakeEigenCommaFormat, Vector) {
  auto eigen_format = beluga_amcl::utils::make_eigen_comma_format();
  std::stringstream out;
  out << Eigen::Vector3d{1.0, 2.0, 3.0}.format(eigen_format);
  ASSERT_EQ(out.str(), "1, 2, 3");
}

TEST(MakeEigenCommaFormat, Matrix) {
  auto eigen_format = beluga_amcl::utils::make_eigen_comma_format();
  std::stringstream out;
  Eigen::Matrix2d matrix;
  matrix << 1.0, 2.0, 3.0, 4.0;
  out << matrix.format(eigen_format);
  ASSERT_EQ(out.str(), "1, 2, 3, 4");
}

TEST(MakePointsFromLaserScan, NoBeams) {
  auto laser_scan = beluga_ros::msg::LaserScan{};
  const auto output = beluga_amcl::utils::make_points_from_laser_scan(laser_scan, Sophus::SE3d{}, 10, 0.0, 20.0);
  ASSERT_EQ(output.size(), 0UL);
}

TEST(MakePointsFromLaserScan, MinRangeInternal) {
  auto laser_scan = beluga_ros::msg::LaserScan{};
  laser_scan.range_min = 0.0;
  laser_scan.range_max = 20.0;
  laser_scan.ranges = std::vector<float>{5, 6, 7, 8, 9};
  const auto output = beluga_amcl::utils::make_points_from_laser_scan(
      laser_scan, Sophus::SE3d{}, laser_scan.ranges.size(), 7.5, laser_scan.range_max);
  ASSERT_EQ(output.size(), 2UL);
}

TEST(MakePointsFromLaserScan, MinRangeExternal) {
  auto laser_scan = beluga_ros::msg::LaserScan{};
  laser_scan.range_min = 7.5;
  laser_scan.range_max = 20.0;
  laser_scan.ranges = std::vector<float>{5, 6, 7, 8, 9};
  const auto output = beluga_amcl::utils::make_points_from_laser_scan(
      laser_scan, Sophus::SE3d{}, laser_scan.ranges.size(), 0.0, laser_scan.range_max);
  ASSERT_EQ(output.size(), 2UL);
}

TEST(MakePointsFromLaserScan, MaxRangeInternal) {
  auto laser_scan = beluga_ros::msg::LaserScan{};
  laser_scan.range_min = 0.0;
  laser_scan.range_max = 7.5;
  laser_scan.ranges = std::vector<float>{5, 6, 7, 8, 9};
  const auto output = beluga_amcl::utils::make_points_from_laser_scan(
      laser_scan, Sophus::SE3d{}, laser_scan.ranges.size(), laser_scan.range_min, 20.0);
  ASSERT_EQ(output.size(), 3UL);
}

TEST(MakePointsFromLaserScan, MaxRangeExternal) {
  auto laser_scan = beluga_ros::msg::LaserScan{};
  laser_scan.range_min = 0.0;
  laser_scan.range_max = 20.0;
  laser_scan.ranges = std::vector<float>{5, 6, 7, 8, 9};
  const auto output = beluga_amcl::utils::make_points_from_laser_scan(
      laser_scan, Sophus::SE3d{}, laser_scan.ranges.size(), laser_scan.range_min, 7.5);
  ASSERT_EQ(output.size(), 3UL);
}

TEST(MakePointsFromLaserScan, MaxBeamsZero) {
  auto laser_scan = beluga_ros::msg::LaserScan{};
  laser_scan.range_min = 0.0;
  laser_scan.range_max = 20.0;
  laser_scan.ranges = std::vector<float>{5, 6, 7, 8, 9, 10};
  const auto output = beluga_amcl::utils::make_points_from_laser_scan(
      laser_scan, Sophus::SE3d{}, 0, laser_scan.range_min, laser_scan.range_max);
  ASSERT_EQ(output.size(), 0UL);
}

TEST(MakePointsFromLaserScan, TakeThreeBeamsFromSix) {
  auto laser_scan = beluga_ros::msg::LaserScan{};
  laser_scan.range_min = 0.0;
  laser_scan.range_max = 20.0;
  laser_scan.ranges = std::vector<float>{5, 6, 7, 8, 9, 10};
  const auto output = beluga_amcl::utils::make_points_from_laser_scan(
      laser_scan, Sophus::SE3d{}, 3, laser_scan.range_min, laser_scan.range_max);
  ASSERT_EQ(output.size(), 3UL);
  EXPECT_NEAR(norm(output[0]), 5.0, 0.001);
  EXPECT_NEAR(norm(output[1]), 7.0, 0.001);
  EXPECT_NEAR(norm(output[2]), 9.0, 0.001);
}

TEST(MakePointsFromLaserScan, TakeThreeBeamsFromNine) {
  auto laser_scan = beluga_ros::msg::LaserScan{};
  laser_scan.range_min = 0.0;
  laser_scan.range_max = 20.0;
  laser_scan.ranges = std::vector<float>{5, 6, 7, 8, 9, 10, 11, 12, 13};
  const auto output = beluga_amcl::utils::make_points_from_laser_scan(
      laser_scan, Sophus::SE3d{}, 3, laser_scan.range_min, laser_scan.range_max);
  ASSERT_EQ(output.size(), 3UL);
  EXPECT_NEAR(norm(output[0]), 5.0, 0.001);
  EXPECT_NEAR(norm(output[1]), 9.0, 0.001);
  EXPECT_NEAR(norm(output[2]), 13.0, 0.001);
}

TEST(MakePointsFromLaserScan, TransformIdentity) {
  auto laser_scan = beluga_ros::msg::LaserScan{};
  laser_scan.range_min = 0.0;
  laser_scan.range_max = 20.0;
  laser_scan.angle_min = Sophus::Constants<float>::pi() / 4;
  laser_scan.angle_max = Sophus::Constants<float>::pi() * 3 / 4;
  laser_scan.angle_increment = Sophus::Constants<float>::pi() / 4;
  laser_scan.time_increment = 0;
  laser_scan.ranges = std::vector<float>{5, 5, 5};
  const auto output = beluga_amcl::utils::make_points_from_laser_scan(
      laser_scan, Sophus::SE3d{}, laser_scan.ranges.size(), laser_scan.range_min, laser_scan.range_max);
  ASSERT_EQ(output.size(), 3UL);
  EXPECT_NEAR(output[0].first, 3.535, 0.001);
  EXPECT_NEAR(output[0].second, 3.535, 0.001);
  EXPECT_NEAR(output[1].first, 0.000, 0.001);
  EXPECT_NEAR(output[1].second, 5.000, 0.001);
  EXPECT_NEAR(output[2].first, -3.535, 0.001);
  EXPECT_NEAR(output[2].second, 3.535, 0.001);
}

TEST(MakePointsFromLaserScan, Transform) {
  auto laser_scan = beluga_ros::msg::LaserScan{};
  laser_scan.range_min = 0.0;
  laser_scan.range_max = 20.0;
  laser_scan.angle_min = Sophus::Constants<float>::pi() / 4;
  laser_scan.angle_max = Sophus::Constants<float>::pi() * 3 / 4;
  laser_scan.angle_increment = Sophus::Constants<float>::pi() / 4;
  laser_scan.time_increment = 0;
  laser_scan.ranges = std::vector<float>{5, 5, 5};
  const auto output = beluga_amcl::utils::make_points_from_laser_scan(
      laser_scan,
      Sophus::SE3d{
          Sophus::SO3d{
              Eigen::AngleAxisd(-Sophus::Constants<float>::pi() * 3 / 4, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(Sophus::Constants<float>::pi() / 4, Eigen::Vector3d::UnitY())},
          Eigen::Vector3d{1.0, 2.0, 3.0}},
      laser_scan.ranges.size(), laser_scan.range_min, laser_scan.range_max);
  ASSERT_EQ(output.size(), 3UL);
  EXPECT_NEAR(output[0].first, 3.500, 0.001);
  EXPECT_NEAR(output[0].second, -2.267, 0.001);
  EXPECT_NEAR(output[1].first, 1.000, 0.001);
  EXPECT_NEAR(output[1].second, -1.535, 0.001);
  EXPECT_NEAR(output[2].first, -1.500, 0.001);
  EXPECT_NEAR(output[2].second, 1.267, 0.001);
}

}  // namespace

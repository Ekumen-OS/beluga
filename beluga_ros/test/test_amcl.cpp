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

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <execution>
#include <memory>
#include <vector>

#include <Eigen/Core>

#if BELUGA_ROS_VERSION == 1
#include <boost/smart_ptr.hpp>
#endif

#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/sensor/likelihood_field_model.hpp>

#include "beluga_ros/amcl.hpp"
#include "beluga_ros/laser_scan.hpp"
#include "beluga_ros/messages.hpp"
#include "beluga_ros/sparse_point_cloud.hpp"

namespace {

auto make_dummy_occupancy_grid() {
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

  return beluga_ros::OccupancyGrid{message};
}

auto make_dummy_laser_scan() {
#if BELUGA_ROS_VERSION == 2
  auto message = std::make_shared<beluga_ros::msg::LaserScan>();
#elif BELUGA_ROS_VERSION == 1
  auto message = boost::make_shared<beluga_ros::msg::LaserScan>();
#endif
  message->ranges = std::vector<float>{1., 2., 3.};
  message->range_min = 10.F;
  message->range_max = 100.F;
  return beluga_ros::LaserScan(message);
}

auto make_dummy_point_cloud() {
#if BELUGA_ROS_VERSION == 2
  auto message = std::make_shared<beluga_ros::msg::PointCloud2>();
#elif BELUGA_ROS_VERSION == 1
  auto message = boost::make_shared<beluga_ros::msg::PointCloud2>();
#else
#error BELUGA_ROS_VERSION is not defined or invalid
#endif
  message->width = 1;
  message->height = 1;
  message->is_dense = true;
  message->is_bigendian = false;

  beluga_ros::msg::PointCloud2Modifier modifier(*message);
  modifier.setPointCloud2Fields(
      3, "x", 1, beluga_ros::msg::PointField::FLOAT64, "y", 1, beluga_ros::msg::PointField::FLOAT64, "z", 1,
      beluga_ros::msg::PointField::FLOAT64);
  modifier.resize(1);

  beluga_ros::msg::PointCloud2Iterator<double> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<double> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<double> iter_z(*message, "z");
  *iter_x = 1.0;
  *iter_y = 0.0;
  *iter_z = 0.0;

  return beluga_ros::SparsePointCloud3<double>{message};
}

auto make_amcl() {
  auto map = make_dummy_occupancy_grid();
  auto params = beluga_ros::AmclParams{};
  params.max_particles = 50UL;
  return beluga_ros::Amcl{
      map,                                                                     //
      beluga::DifferentialDriveModel{beluga::DifferentialDriveModelParam{}},   //
      beluga::LikelihoodFieldModel{beluga::LikelihoodFieldModelParam{}, map},  //
      params,                                                                  //
      std::execution::seq,
  };
}

TEST(TestAmcl, InitializeWithNoParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
}

TEST(TestAmcl, InitializeFromMap) {
  auto amcl = make_amcl();
  amcl.initialize_from_map();
  ASSERT_EQ(amcl.particles().size(), 50UL);
}

TEST(TestAmcl, InitializeFromPose) {
  auto amcl = make_amcl();
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), 50UL);
}

TEST(TestAmcl, UpdateWithNoParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  auto estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_FALSE(estimate.has_value());
}

TEST(TestAmcl, UpdateWithParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize_from_map();
  ASSERT_EQ(amcl.particles().size(), 50UL);
  auto estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_TRUE(estimate.has_value());
}

TEST(TestAmcl, UpdateWithParticlesAndPointCloud) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize_from_map();
  ASSERT_EQ(amcl.particles().size(), 50UL);
  auto estimate = amcl.update(Sophus::SE2d{}, make_dummy_point_cloud());
  ASSERT_TRUE(estimate.has_value());
}

TEST(TestAmcl, UpdateWithParticlesWithMotion) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize_from_map();
  ASSERT_EQ(amcl.particles().size(), 50UL);
  auto estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_TRUE(estimate.has_value());
  estimate = amcl.update(Sophus::SE2d{0.0, {1.0, 0.0}}, make_dummy_laser_scan());
  ASSERT_TRUE(estimate.has_value());
}

TEST(TestAmcl, UpdateWithParticlesNoMotion) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize_from_map();
  ASSERT_EQ(amcl.particles().size(), 50UL);
  auto estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_TRUE(estimate.has_value());
  estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_FALSE(estimate.has_value());
}

TEST(TestAmcl, UpdateWithParticlesForced) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize_from_map();
  ASSERT_EQ(amcl.particles().size(), 50UL);
  auto estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_TRUE(estimate.has_value());
  amcl.force_update();
  estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_TRUE(estimate.has_value());
}

}  // namespace

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

#include <gmock/gmock.h>

#include <beluga_ros/amcl_impl.hpp>

#if BELUGA_ROS_VERSION == 1
#include <boost/smart_ptr.hpp>
#endif

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

auto make_amcl() {
  auto map = make_dummy_occupancy_grid();
  auto params = beluga_ros::AmclImplParams{};
  params.max_particles = 50UL;
  return beluga_ros::AmclImpl{
      params,                                                                                             //
      map,                                                                                                //
      beluga::DifferentialDriveModel{beluga::DifferentialDriveModelParam{}},                              //
      beluga::LikelihoodFieldModel<beluga_ros::OccupancyGrid>{beluga::LikelihoodFieldModelParam{}, map},  //
      std::execution::seq,
  };
}

TEST(TestAmclImpl, InitializeWithNoParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
}

TEST(TestAmclImpl, InitializeFromMap) {
  auto amcl = make_amcl();
  amcl.initialize_from_map();
  ASSERT_EQ(amcl.particles().size(), 50UL);
}

TEST(TestAmclImpl, InitializeFromPose) {
  auto amcl = make_amcl();
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), 50UL);
}

TEST(TestAmclImpl, UpdateWithNoParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  auto estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_FALSE(estimate.has_value());
}

TEST(TestAmclImpl, UpdateWithParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize_from_map();
  ASSERT_EQ(amcl.particles().size(), 50UL);
  auto estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_TRUE(estimate.has_value());
}

TEST(TestAmclImpl, UpdateWithParticlesNoMotion) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize_from_map();
  ASSERT_EQ(amcl.particles().size(), 50UL);
  auto estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_TRUE(estimate.has_value());
  estimate = amcl.update(Sophus::SE2d{}, make_dummy_laser_scan());
  ASSERT_FALSE(estimate.has_value());
}

TEST(TestAmclImpl, UpdateWithParticlesForced) {
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

// Copyright 2026 Ekumen, Inc.
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

#include <chrono>
#include <thread>

#include <gmock/gmock.h>

#include <rclcpp/rclcpp.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation_ros/converters/clock.hpp>
#include <beluga_estimation_ros/interfaces/middleware.hpp>
#include <beluga_estimation_ros/interfaces/transform.hpp>

namespace {

class TransformTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TransformTest, LookupTransformNotFound) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  beluga_estimation_ros::TF2Interface interface {
    beluga_estimation_ros::MakeNodeInterface(node)
  };

  auto time = beluga_estimation_ros::FromRos(node->now());
  auto result = interface.LookupTransform("map", "odom", time);

  EXPECT_FALSE(result.has_value());
}

TEST_F(TransformTest, BroadcastAndLookupTransform) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  beluga_estimation_ros::TF2Interface interface {
    beluga_estimation_ros::MakeNodeInterface(node)
  };

  auto time = beluga_estimation_ros::FromRos(node->now());
  beluga_estimation::SE3d expected_transform{
      Eigen::Quaterniond{Eigen::AngleAxisd{M_PI / 2, Eigen::Vector3d::UnitZ()}}, Eigen::Vector3d{1.0, 2.0, 3.0}};

  interface.BroadcastTransform("map", "odom", time, expected_transform);

  std::optional<beluga_estimation::SE3d> result;
  auto start = std::chrono::steady_clock::now();

  // Spin the node to process the broadcasted transform via the ROS 2 tf topic.
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
    rclcpp::spin_some(node);
    result = interface.LookupTransform("map", "odom", time);
    if (result.has_value()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(result.has_value());
  constexpr double kTolerance = 1e-4;
  EXPECT_NEAR(result->translation().x(), 1.0, kTolerance);
  EXPECT_NEAR(result->translation().y(), 2.0, kTolerance);
  EXPECT_NEAR(result->translation().z(), 3.0, kTolerance);
  EXPECT_NEAR(result->so3().log().x(), 0.0, kTolerance);
  EXPECT_NEAR(result->so3().log().y(), 0.0, kTolerance);
  EXPECT_NEAR(result->so3().log().z(), M_PI / 2, kTolerance);
}

}  // namespace

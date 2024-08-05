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

#include <gmock/gmock-spec-builders.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/utilities.hpp>

#include <sophus/common.hpp>
#include <thread>

#include "beluga_amcl/amcl_node.hpp"
#include "beluga_amcl/ros2_common.hpp"
#include "test_utils/node_testing.hpp"

namespace beluga_amcl {

class MockAMCL : public BaseAMCLNode {
 public:
  using BaseAMCLNode::BaseAMCLNode;
  MOCK_METHOD(void, do_activate, ([[maybe_unused]] const rclcpp_lifecycle::State& state), (override));
  MOCK_METHOD(void, do_configure, ([[maybe_unused]] const rclcpp_lifecycle::State& state), (override));
  MOCK_METHOD(void, do_shutdown, ([[maybe_unused]] const rclcpp_lifecycle::State& state), (override));
  MOCK_METHOD(void, do_deactivate, ([[maybe_unused]] const rclcpp_lifecycle::State& state), (override));
  MOCK_METHOD(void, do_cleanup, ([[maybe_unused]] const rclcpp_lifecycle::State& state), (override));
  MOCK_METHOD(void, do_autostart_callback, (), (override));
  MOCK_METHOD(void, do_periodic_timer_callback, (), (override));
};

class TestROS2Common : public ::testing::Test {
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TestROS2Common, Configure) {
  using ::testing::_;
  auto amcl = std::make_shared<MockAMCL>();
  EXPECT_CALL(*amcl, do_configure(_));
  amcl->configure();
}

TEST_F(TestROS2Common, Activate) {
  using ::testing::_;
  auto amcl = std::make_shared<MockAMCL>();
  EXPECT_CALL(*amcl, do_activate(_));
  EXPECT_CALL(*amcl, do_configure(_));
  amcl->configure();
  amcl->activate();
}

TEST_F(TestROS2Common, CleanupDeactivateShutdownDefaults) {
  using ::testing::_;
  auto amcl = std::make_shared<BaseAMCLNode>();
  amcl->configure();
  amcl->activate();
  amcl->deactivate();
  amcl->cleanup();
  amcl->shutdown();
}

TEST_F(TestROS2Common, CleanupDeactivateShutdownUserProvided) {
  using ::testing::_;
  auto amcl = std::make_shared<MockAMCL>();
  EXPECT_CALL(*amcl, do_configure(_));
  EXPECT_CALL(*amcl, do_activate(_));
  EXPECT_CALL(*amcl, do_deactivate(_));
  EXPECT_CALL(*amcl, do_shutdown(_));
  EXPECT_CALL(*amcl, do_cleanup(_));
  amcl->configure();
  amcl->activate();
  amcl->deactivate();
  amcl->cleanup();
  amcl->shutdown();
}

TEST_F(TestROS2Common, Autostart) {
  using namespace std::chrono_literals;
  auto amcl = std::make_shared<MockAMCL>(
      "amcl", "",
      rclcpp::NodeOptions{}
          .append_parameter_override("autostart", true)
          .append_parameter_override("autostart_delay", 0.1));
  EXPECT_CALL(*amcl, do_autostart_callback());
  testing::spin_for(300ms, amcl);
}

TEST_F(TestROS2Common, PeriodicTimer) {
  using namespace std::chrono_literals;
  using ::testing::_;
  auto amcl =
      std::make_shared<MockAMCL>("amcl", "", rclcpp::NodeOptions{}.append_parameter_override("autostart", false));
  EXPECT_CALL(*amcl, do_periodic_timer_callback());
  EXPECT_CALL(*amcl, do_configure(_));
  EXPECT_CALL(*amcl, do_activate(_));
  amcl->configure();
  amcl->activate();
  testing::spin_for(300ms, amcl);
}

}  // namespace beluga_amcl

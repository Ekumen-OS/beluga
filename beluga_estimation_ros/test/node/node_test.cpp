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
#include <memory>
#include <string>
#include <thread>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <beluga_estimation/beluga_estimation.hpp>
#include <beluga_estimation_ros/beluga_ros.hpp>
#include <beluga_estimation_ros/node.hpp>

namespace {

using namespace std::chrono_literals;

class NodeTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::NodeOptions MakeNodeOptions(const std::string& method_type, bool managed = false) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("method.type", method_type);
    options.append_parameter_override("managed", managed);
    options.append_parameter_override("heartbeat.enable", false);
    return options;
  }
};

TEST_F(NodeTest, UnmanagedModeAutoActivates) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodSlow", false);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  // In unmanaged mode, the node should auto-configure and auto-activate
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(NodeTest, ManagedModeRequiresManualTransitions) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodSlow", true);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  // In managed mode, the node should remain in unconfigured state
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Configure the node
  node->configure();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Activate the node
  node->activate();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(NodeTest, InvalidMethodTypeFailsConfiguration) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::NonExistentMethod", true);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  // Configuration fails when the method type is not found in the registry.
  node->configure();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(NodeTest, MethodExecutesInUnmanagedMode) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodQuick", false);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  // In unmanaged mode, the node should auto-activate
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Give the method thread time to execute and complete
  std::this_thread::sleep_for(100ms);

  // After the method completes successfully, the node should deactivate
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(NodeTest, ManagedModeLifecycleTransitions) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodSlow", true);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  // Initial state
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Configure
  node->configure();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Activate
  node->activate();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Give the method thread time to start
  std::this_thread::sleep_for(100ms);

  // Deactivate
  node->deactivate();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Cleanup
  node->cleanup();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(NodeTest, MethodCompletionTriggersDeactivation) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodQuick", true);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  node->configure();
  node->activate();

  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Give the method thread time to execute and complete
  std::this_thread::sleep_for(100ms);

  // After the method completes successfully, the node should auto-deactivate
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(NodeTest, DeactivateStopsMethodExecution) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodSlow", true);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  node->configure();
  node->activate();

  // Give the method thread time to start
  std::this_thread::sleep_for(100ms);

  // Deactivate should request stop and join the thread
  auto deactivate_start = std::chrono::steady_clock::now();
  node->deactivate();
  auto deactivate_duration = std::chrono::steady_clock::now() - deactivate_start;

  // Deactivation should complete relatively quickly (< 1 second)
  // because the method should honor the stop token
  EXPECT_LT(deactivate_duration, 1s);

  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(NodeTest, ShutdownFromActiveState) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodSlow", true);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  node->configure();
  node->activate();

  // Give the method thread time to start
  std::this_thread::sleep_for(100ms);

  // Shutdown from active state should clean up properly
  auto shutdown_start = std::chrono::steady_clock::now();
  node->shutdown();
  auto shutdown_duration = std::chrono::steady_clock::now() - shutdown_start;

  // Shutdown should complete relatively quickly
  EXPECT_LT(shutdown_duration, 1s);

  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

TEST_F(NodeTest, ShutdownFromInactiveState) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodQuick", true);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  node->configure();

  // Shutdown from inactive state
  node->shutdown();

  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

TEST_F(NodeTest, ActivateDeactivateActivateCycle) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodSlow", true);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  node->configure();

  // First activation
  node->activate();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  std::this_thread::sleep_for(100ms);

  // Deactivation
  node->deactivate();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Second activation
  node->activate();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  std::this_thread::sleep_for(100ms);

  // Final deactivation
  node->deactivate();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(NodeTest, ConfigureCleanupConfigureCycle) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodSlow", true);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  // First configuration
  node->configure();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Cleanup
  node->cleanup();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Second configuration
  node->configure();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Activate to verify it still works
  node->activate();
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(NodeTest, MethodFailureTriggersShutdown) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodFailing", true);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  node->configure();
  node->activate();

  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Give time for the failing method to execute and trigger shutdown
  std::this_thread::sleep_for(200ms);

  // After the method fails, the node should transition to finalized state
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

TEST_F(NodeTest, ParametersAreDeclared) {
  auto options = MakeNodeOptions("beluga_estimation_ros::test::MockMethodSlow", false);
  auto node = std::make_shared<beluga_estimation_ros::Node>(options);

  // Verify parameters are declared
  EXPECT_TRUE(node->has_parameter("method.type"));
  EXPECT_TRUE(node->has_parameter("managed"));
  EXPECT_TRUE(node->has_parameter("heartbeat.rate"));
  EXPECT_TRUE(node->has_parameter("heartbeat.timeout"));

  // Verify parameter values
  EXPECT_EQ(node->get_parameter("method.type").as_string(), "beluga_estimation_ros::test::MockMethodSlow");
  EXPECT_EQ(node->get_parameter("managed").as_bool(), false);
  EXPECT_DOUBLE_EQ(node->get_parameter("heartbeat.rate").as_double(), 10.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("heartbeat.timeout").as_double(), 5.0);
}

#ifdef BELUGA_ESTIMATION_ROS_TEST_CONFIG_PATH
TEST_F(NodeTest, DefaultConfigurationFileLoadsSuccessfully) {
  rclcpp::NodeOptions options;
  options.arguments({
      "--ros-args",
      "-r",
      "__node:=beluga_filter_node",
      "--params-file",
      BELUGA_ESTIMATION_ROS_TEST_CONFIG_PATH,
  });
  options.append_parameter_override("managed", true);

  auto node = std::make_shared<beluga_estimation_ros::Node>(options);
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Configuring the node instantiates beluga_estimation::RecursiveFilter and loads all stages from the YAML file.
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  EXPECT_TRUE(node->has_parameter("method.rate"));
  EXPECT_DOUBLE_EQ(node->get_parameter("method.rate").as_double(), 10.0);
}
#endif

}  // namespace

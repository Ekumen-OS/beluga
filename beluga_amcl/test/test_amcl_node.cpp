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

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <tuple>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/utilities.hpp>

#include <sophus/common.hpp>

#include "beluga_amcl/amcl_node.hpp"
#include "test_utils/node_testing.hpp"

namespace {

using namespace std::chrono_literals;
using beluga_amcl::testing::spin_for;
using beluga_amcl::testing::spin_until;

/// Test class that provides convenient public accessors.
class AmclNodeUnderTest : public beluga_amcl::AmclNode {
 public:
  /// Get particle filter pointer.
  const auto& particle_filter() { return particle_filter_; }

  /// Return true if the particle filter has been initialized.
  bool is_initialized() const { return particle_filter_ != nullptr; }

  /// Return the last known estimate. Throws if there is no estimate.
  const auto& estimate() { return last_known_estimate_.value(); }
};

/// Base node fixture class with common utilities.
template <class T>
class BaseNodeFixture : public T {
 public:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    amcl_node_ = std::make_shared<AmclNodeUnderTest>();
    tester_node_ = std::make_shared<beluga_amcl::testing::TesterNode>();
    tester_node_->create_transform_buffer();
  }

  void TearDown() override { rclcpp::shutdown(); }

  bool wait_for_initialization() {
    return spin_until([this] { return amcl_node_->is_initialized(); }, 200ms, amcl_node_, tester_node_);
  }

  bool wait_for_pose_estimate() {
    tester_node_->latest_pose().reset();
    tester_node_->create_pose_subscriber();
    return spin_until([this] { return tester_node_->latest_pose().has_value(); }, 1000ms, amcl_node_, tester_node_);
  }

  bool wait_for_transform(const std::string& target, const std::string& source) {
    return spin_until(
        [&, this] { return tester_node_->can_transform(target, source); }, 1000ms, amcl_node_, tester_node_);
  }

  bool wait_for_particle_cloud() {
    tester_node_->create_particle_cloud_subscriber();
    return spin_until(
        [this] { return tester_node_->latest_particle_cloud().has_value(); }, 1000ms, amcl_node_, tester_node_);
  }

  bool wait_for_particle_markers() {
    tester_node_->create_particle_markers_subscriber();
    return spin_until(
        [this] { return tester_node_->latest_particle_markers().has_value(); }, 1000ms, amcl_node_, tester_node_);
  }

  bool wait_for_likelihood_field() {
    tester_node_->create_likelihood_field_subscriber();
    return spin_until(
        [this] { return tester_node_->latest_likelihood_field().has_value(); }, 1000ms, amcl_node_, tester_node_);
  }

  bool request_global_localization() {
    if (!tester_node_->wait_for_global_localization_service(500ms)) {
      return false;
    }
    auto future = tester_node_->async_request_global_localization();
    const bool done =
        spin_until([&] { return future.wait_for(0s) == std::future_status::ready; }, 500ms, amcl_node_, tester_node_);
    if (done) {
      return true;
    }
    tester_node_->prune_pending_global_localization_requests();
    return false;
  }

  bool request_nomotion_update() {
    if (!tester_node_->wait_for_global_localization_service(500ms)) {
      return false;
    }
    auto future = tester_node_->async_nomotion_update_request();
    const bool done =
        spin_until([&] { return future.wait_for(0s) == std::future_status::ready; }, 500ms, amcl_node_, tester_node_);
    if (done) {
      return true;
    }
    tester_node_->prune_pending_nomotion_update_request();
    return false;
  }

 protected:
  std::shared_ptr<AmclNodeUnderTest> amcl_node_;
  std::shared_ptr<beluga_amcl::testing::TesterNode> tester_node_;
};

class TestLifecycle : public ::testing::Test {
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TestLifecycle, FullSpin) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, ShutdownWhenActive) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, ShutdownWhenInactive) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, ShutdownWhenUnconfigured) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, DestroyWhenActive) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TestLifecycle, DestroyWhenInactive) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(TestLifecycle, AutoStart) {
  using lifecycle_msgs::msg::State;
  auto node =
      std::make_shared<beluga_amcl::AmclNode>(rclcpp::NodeOptions{}.append_parameter_override("autostart", true));
  spin_until([&] { return node->get_current_state().id() == State::PRIMARY_STATE_ACTIVE; }, 100ms, node);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

class TestInitializationWithModel
    : public BaseNodeFixture<::testing::TestWithParam<std::tuple<const char*, const char*>>> {};

INSTANTIATE_TEST_SUITE_P(
    Models,
    TestInitializationWithModel,
    testing::Values(
        std::make_tuple("differential_drive", "likelihood_field_prob"),
        std::make_tuple("differential_drive", "likelihood_field"),
        std::make_tuple("omnidirectional_drive", "beam"),
        std::make_tuple("stationary", "likelihood_field")));

TEST_P(TestInitializationWithModel, ParticleCount) {
  const auto [motion_model, sensor_model] = GetParam();

  amcl_node_->set_parameter(rclcpp::Parameter{"robot_model_type", motion_model});
  amcl_node_->set_parameter(rclcpp::Parameter{"laser_model_type", sensor_model});
  amcl_node_->set_parameter(rclcpp::Parameter{"min_particles", 10});
  amcl_node_->set_parameter(rclcpp::Parameter{"max_particles", 30});

  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());

  ASSERT_GE(amcl_node_->particle_filter()->particles().size(), 10UL);
  ASSERT_LE(amcl_node_->particle_filter()->particles().size(), 30UL);
}

class TestNode : public BaseNodeFixture<::testing::Test> {};

TEST_F(TestNode, MapWithWrongFrame) {
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map_with_wrong_frame();
  ASSERT_TRUE(wait_for_initialization());
}

TEST_F(TestNode, SetInitialPose) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  const auto [pose, _] = amcl_node_->estimate();
  ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
  ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
}

TEST_F(TestNode, BroadcastWhenInitialPoseSet) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_transform("map", "odom"));
}

TEST_F(TestNode, NoBroadcastWhenNoInitialPose) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_laser_scan();
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, BroadcastWithGlobalLocalization) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  ASSERT_TRUE(request_global_localization());
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_transform("map", "odom"));
}

TEST_F(TestNode, IgnoreGlobalLocalizationBeforeInitialize) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  amcl_node_->configure();
  amcl_node_->activate();
  ASSERT_TRUE(request_global_localization());
  tester_node_->publish_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
}

TEST_F(TestNode, NoBroadcastWhenInitialPoseInvalid) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.0});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.0});
  amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", -50.0});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, FirstMapOnly) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node_->set_parameter(rclcpp::Parameter{"always_reset_initial_pose", true});

  {
    // Set initial pose values to simulate an estimate that has converged.
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  }

  amcl_node_->set_parameter(rclcpp::Parameter{"first_map_only", true});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());

  {
    // Initialized with the first map and initial pose values.
    const auto [pose, _] = amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
  }

  {
    // Set new initial pose values that will be ignored.
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 1.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 29.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", -0.4});
  }

  tester_node_->publish_map();
  spin_for(50ms, tester_node_, amcl_node_);

  {
    // Ignored the new initial pose values (and map).
    const auto [pose, _] = amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
  }

  amcl_node_->set_parameter(rclcpp::Parameter{"first_map_only", false});
  tester_node_->publish_map();
  spin_for(50ms, tester_node_, amcl_node_);

  {
    // Initialized with the new initial pose values (and map).
    const auto [pose, _] = amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 1.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 29.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), -0.4, 0.01);
  }
}

TEST_F(TestNode, KeepCurrentEstimate) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node_->set_parameter(rclcpp::Parameter{"always_reset_initial_pose", false});
  amcl_node_->set_parameter(rclcpp::Parameter{"first_map_only", false});

  {
    // Set initial pose values to simulate an estimate that has converged.
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  }

  amcl_node_->configure();
  amcl_node_->activate();

  {
    // Initializing with the first map and initial pose values.
    tester_node_->publish_map();
    ASSERT_TRUE(wait_for_initialization());
    const auto [pose, _] = amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
  }

  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  const auto [estimate, _] = amcl_node_->estimate();

  {
    // Set new initial pose values that will be ignored.
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 1.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 29.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", -0.4});
  }

  tester_node_->publish_map();
  spin_for(50ms, tester_node_, amcl_node_);

  {
    // Initializing with the second map but keeping the old estimate.
    // Ignoring the new initial pose values.
    const auto [pose, _] = amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), estimate.translation().x(), 0.01);
    ASSERT_NEAR(pose.translation().y(), estimate.translation().y(), 0.01);
    ASSERT_NEAR(pose.so2().log(), estimate.so2().log(), 0.01);
  }
}

TEST_F(TestNode, KeepCurrentEstimateAfterCleanup) {
  amcl_node_->set_parameter(rclcpp::Parameter{"always_reset_initial_pose", false});
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});

  {
    // Set initial pose values to simulate an estimate that has converged.
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  }

  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  amcl_node_->deactivate();
  amcl_node_->cleanup();

  {
    // Set new initial pose values that will be ignored.
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 12.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 1.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.7});
  }

  amcl_node_->configure();
  amcl_node_->activate();
  ASSERT_EQ(amcl_node_->particle_filter().get(), nullptr);
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  const auto [pose, _] = amcl_node_->estimate();
  ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
  ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
}

TEST_F(TestNode, InvalidMotionModel) {
  amcl_node_->set_parameter(rclcpp::Parameter{"robot_model_type", "non_existing_model"});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_FALSE(wait_for_initialization());
}

TEST_F(TestNode, InvalidSensorModel) {
  amcl_node_->set_parameter(rclcpp::Parameter{"laser_model_type", "non_existing_model"});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_FALSE(wait_for_initialization());
}

TEST_F(TestNode, InvalidExecutionPolicy) {
  amcl_node_->set_parameter(rclcpp::Parameter{"execution_policy", "non_existing_policy"});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_FALSE(wait_for_initialization());
}

TEST_F(TestNode, SequentialExecutionPolicy) {
  amcl_node_->set_parameter(rclcpp::Parameter{"execution_policy", "seq"});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
}

TEST_F(TestNode, ParallelExecutionPolicy) {
  amcl_node_->set_parameter(rclcpp::Parameter{"execution_policy", "par"});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
}

TEST_F(TestNode, InitialPoseBeforeInitialize) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_default_initial_pose();
  spin_for(10ms, amcl_node_);  // ensure orderly processing
  tester_node_->publish_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
}

TEST_F(TestNode, InitialPoseAfterInitialize) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_default_initial_pose();
  spin_for(10ms, amcl_node_);  // ensure orderly processing
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(wait_for_transform("map", "odom"));
}

TEST_F(TestNode, InitialPoseWithWrongFrame) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_initial_pose_with_wrong_frame();
  spin_for(10ms, amcl_node_);  // ensure orderly processing
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, CanUpdatePoseEstimate) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_default_initial_pose();
  spin_for(10ms, amcl_node_);  // ensure orderly processing
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(wait_for_transform("map", "odom"));

  {
    const auto [pose, _] = amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 0.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 0.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0., 0.01);
  }

  tester_node_->publish_initial_pose(1., 1.);
  spin_for(10ms, amcl_node_);  // ensure orderly processing
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(wait_for_transform("map", "odom"));

  {
    const auto [pose, _] = amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 1.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 1.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0., 0.01);
  }
}

TEST_F(TestNode, CanUpdatePoseEstimateWithPointCloud) {
  amcl_node_->set_parameter(rclcpp::Parameter{"point_cloud_topic", "point_cloud"});
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_default_initial_pose();
  spin_for(10ms, amcl_node_, tester_node_);  // ensure orderly processing
  tester_node_->publish_point_cloud();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(wait_for_transform("map", "odom"));
}

TEST_F(TestNode, ThrowsIfBothSensorTopicsAreSet) {
  amcl_node_->set_parameter(rclcpp::Parameter{"scan_topic", "scan"});
  amcl_node_->set_parameter(rclcpp::Parameter{"point_cloud_topic", "point_cloud"});
  amcl_node_->configure();

  // std::invalid argument exception launched on AmclNode::do_activate is not visible at this stage
  // (catched by callback method. See rclcpp_lifecycle).
  //
  // Testing failure is evidenced evaluating node state is not ACTIVE after activating.

  // Activate node, expecting no exceptions thrown
  EXPECT_NO_THROW(amcl_node_->activate());

  // Check lifecycle state after activation — expect INACTIVE state (Activation didn't succeded)
  auto state = amcl_node_->get_current_state();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(TestNode, CanForcePoseEstimate) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_default_initial_pose();
  spin_for(10ms, amcl_node_);  // ensure orderly processing
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(wait_for_transform("map", "odom"));

  {
    const auto [pose, _] = amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 0.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 0.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0., 0.01);
  }

  ASSERT_TRUE(request_nomotion_update());
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(wait_for_transform("map", "odom"));

  {
    const auto [pose, _] = amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 0.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 0.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0., 0.01);
  }
}

TEST_F(TestNode, IgnoreNoMotionUpdateBeforeInitializeAkaTheCodeCoverageTest) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  amcl_node_->configure();
  amcl_node_->activate();
  ASSERT_TRUE(request_nomotion_update());
  tester_node_->publish_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
}

TEST_F(TestNode, IsPublishingParticleCloud) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_TRUE(wait_for_particle_cloud());
}

TEST_F(TestNode, IsPublishingParticleMarkers) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_TRUE(wait_for_particle_markers());
}

TEST_F(TestNode, IsPublishingLikelihoodField) {
  amcl_node_->set_parameter(rclcpp::Parameter{"laser_model_type", "likelihood_field"});
  amcl_node_->set_parameter(rclcpp::Parameter{"debug", true});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_TRUE(wait_for_likelihood_field());
}

TEST_F(TestNode, LaserScanWithNoOdomToBase) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->publish_laser_scan_with_no_odom_to_base();
  ASSERT_FALSE(wait_for_pose_estimate());
}

TEST_F(TestNode, TransformValue) {
  amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node_->set_parameter(rclcpp::Parameter{"always_reset_initial_pose", true});

  {
    // Set initial pose values to simulate an estimate that has converged.
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 1.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", Sophus::Constants<double>::pi() / 3});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
    amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  }

  amcl_node_->configure();
  amcl_node_->activate();
  tester_node_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->publish_laser_scan_with_odom_to_base(Sophus::SE2d{
      Sophus::SO2d{Sophus::Constants<double>::pi() / 3},
      Sophus::Vector2d{3., 4.},
  });
  ASSERT_TRUE(wait_for_pose_estimate());

  const auto [pose, _] = amcl_node_->estimate();
  ASSERT_NEAR(pose.translation().x(), 1.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
  ASSERT_NEAR(pose.so2().log(), Sophus::Constants<double>::pi() / 3, 0.01);

  ASSERT_TRUE(wait_for_transform("map", "odom"));
  const auto transform = tester_node_->lookup_transform("map", "odom");
  EXPECT_NEAR(transform.translation().x(), -2.00, 0.01);
  EXPECT_NEAR(transform.translation().y(), -2.00, 0.01);
  EXPECT_NEAR(transform.so2().log(), 0.0, 0.01);
}

class TestParameterValue : public ::testing::TestWithParam<rclcpp::Parameter> {};

INSTANTIATE_TEST_SUITE_P(
    InvalidParameterValues,
    TestParameterValue,
    testing::Values(
        rclcpp::Parameter("min_particles", -1),
        rclcpp::Parameter("max_particles", -1),
        rclcpp::Parameter("recovery_alpha_slow", -1.0),
        rclcpp::Parameter("recovery_alpha_slow", 2.0),
        rclcpp::Parameter("recovery_alpha_fast", -1.0),
        rclcpp::Parameter("recovery_alpha_fast", 2.0),
        rclcpp::Parameter("pf_err", -1.0),
        rclcpp::Parameter("pf_err", 2.0),
        rclcpp::Parameter("spatial_resolution_x", -1.0),
        rclcpp::Parameter("spatial_resolution_y", -1.0),
        rclcpp::Parameter("spatial_resolution_theta", -1.0),
        rclcpp::Parameter("spatial_resolution_theta", 7.0),
        rclcpp::Parameter("resample_interval", 0),
        rclcpp::Parameter("transform_tolerance", -1.0),
        rclcpp::Parameter("alpha1", -1.0),
        rclcpp::Parameter("alpha2", -1.0),
        rclcpp::Parameter("alpha3", -1.0),
        rclcpp::Parameter("alpha4", -1.0),
        rclcpp::Parameter("alpha5", -1.0),
        rclcpp::Parameter("update_min_a", -1.0),
        rclcpp::Parameter("update_min_a", 7.0),
        rclcpp::Parameter("update_min_d", -1.0),
        rclcpp::Parameter("laser_likelihood_max_dist", -1.0),
        rclcpp::Parameter("laser_max_range", -1.0),
        rclcpp::Parameter("laser_min_range", -1.0),
        rclcpp::Parameter("max_beams", 1),
        rclcpp::Parameter("z_hit", -1.0),
        rclcpp::Parameter("z_hit", 2.0),
        rclcpp::Parameter("z_rand", -1.0),
        rclcpp::Parameter("z_rand", 2.0),
        rclcpp::Parameter("model_unknown_space", false),
        rclcpp::Parameter("model_unknown_space", true),
        rclcpp::Parameter("likelihood_from_strict_obstacle_edges", false),
        rclcpp::Parameter("likelihood_from_strict_obstacle_edges", true),
        rclcpp::Parameter("z_max", -1.0),
        rclcpp::Parameter("z_max", 2.0),
        rclcpp::Parameter("z_short", -1.0),
        rclcpp::Parameter("z_short", 2.0),
        rclcpp::Parameter("lambda_short", -1.0),
        rclcpp::Parameter("sigma_hit", -1.0)));

TEST_P(TestParameterValue, InvalidValue) {
  const auto options = rclcpp::NodeOptions().parameter_overrides({GetParam()});
  ASSERT_ANY_THROW(std::make_shared<beluga_amcl::AmclNode>(options));
}

}  // namespace

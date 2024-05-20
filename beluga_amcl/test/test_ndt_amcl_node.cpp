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

#include <beluga_amcl/ndt_amcl_node.hpp>

#include "test_utils/node_testing.hpp"

#include <gmock/gmock.h>

#include <lifecycle_msgs/msg/state.hpp>

#include <tf2_ros/create_timer_ros.h>

#include <cstdlib>

#include <beluga_ros/tf2_sophus.hpp>

namespace {

const std::string kTestMapPath = "./test_data/turtlebot3_world.hdf5";

using namespace std::chrono_literals;
using beluga_amcl::testing::spin_for;
using beluga_amcl::testing::spin_until;

/// Test class that provides convenient public accessors.
class AmclNodeUnderTest : public beluga_amcl::NdtAmclNode {
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
    ndt_amcl_node_ = std::make_shared<AmclNodeUnderTest>();
    tester_node_ = std::make_shared<beluga_amcl::testing::TesterNode>();
    tester_node_->create_transform_buffer();
    ndt_amcl_node_->set_parameter(rclcpp::Parameter("map_path", kTestMapPath));
  }

  void TearDown() override { rclcpp::shutdown(); }

  bool wait_for_initialization() {
    return spin_until([this] { return ndt_amcl_node_->is_initialized(); }, 200ms, ndt_amcl_node_, tester_node_);
  }

  bool wait_for_pose_estimate() {
    tester_node_->latest_pose().reset();
    tester_node_->create_pose_subscriber();
    return spin_until([this] { return tester_node_->latest_pose().has_value(); }, 1000ms, ndt_amcl_node_, tester_node_);
  }

  bool wait_for_particle_cloud() {
    tester_node_->create_particle_cloud_subscriber();
    return spin_until(
        [this] { return tester_node_->latest_particle_cloud().has_value(); }, 1000ms, ndt_amcl_node_, tester_node_);
  }

 protected:
  std::shared_ptr<AmclNodeUnderTest> ndt_amcl_node_;
  std::shared_ptr<beluga_amcl::testing::TesterNode> tester_node_;
};

class TestLifecycle : public ::testing::Test {
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TestLifecycle, FullSpin) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtAmclNode>();
  node->set_parameter(rclcpp::Parameter("map_path", kTestMapPath));
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
  auto node = std::make_shared<beluga_amcl::NdtAmclNode>();
  node->set_parameter(rclcpp::Parameter("map_path", kTestMapPath));
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
  auto node = std::make_shared<beluga_amcl::NdtAmclNode>();
  node->set_parameter(rclcpp::Parameter("map_path", kTestMapPath));
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, ShutdownWhenUnconfigured) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtAmclNode>();
  node->set_parameter(rclcpp::Parameter("map_path", kTestMapPath));
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, DestroyWhenActive) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtAmclNode>();
  node->set_parameter(rclcpp::Parameter("map_path", kTestMapPath));
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TestLifecycle, DestroyWhenInactive) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtAmclNode>();
  node->set_parameter(rclcpp::Parameter("map_path", kTestMapPath));
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(TestLifecycle, DestroyWhenUnconfigured) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtAmclNode>();
  node->set_parameter(rclcpp::Parameter("map_path", kTestMapPath));
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(TestLifecycle, AutoStart) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtAmclNode>(rclcpp::NodeOptions{}
                                                             .append_parameter_override("autostart", true)
                                                             .append_parameter_override("map_path", kTestMapPath));
  spin_until([&] { return node->get_current_state().id() == State::PRIMARY_STATE_ACTIVE; }, 100ms, node);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

class TestInitializationWithModel : public BaseNodeFixture<::testing::TestWithParam<const char*>> {};

INSTANTIATE_TEST_SUITE_P(
    Models,
    TestInitializationWithModel,
    testing::Values("differential_drive", "omnidirectional_drive", "stationary"));

TEST_P(TestInitializationWithModel, ParticleCount) {
  const auto* const motion_model = GetParam();

  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"robot_model_type", motion_model});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"min_particles", 10});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"max_particles", 30});

  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();

  ASSERT_TRUE(wait_for_initialization());

  std::visit(
      [](const auto& pf) {
        ASSERT_GE(pf.particles().size(), 10UL);
        ASSERT_LE(pf.particles().size(), 30UL);
      },
      *ndt_amcl_node_->particle_filter());
}

class TestNode : public BaseNodeFixture<::testing::Test> {};

TEST_F(TestNode, SetInitialPose) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  const auto [pose, _] = ndt_amcl_node_->estimate();
  ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
  ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
}

TEST_F(TestNode, BroadcastWhenInitialPoseSet) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, NoBroadcastWhenNoInitialPose) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, NoBroadcastWhenInitialPoseInvalid) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.0});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.0});
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", -50.0});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, KeepCurrentEstimate) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  {
    // Set initial pose values to simulate an estimate that has converged.
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  }

  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();

  {
    // Initializing with the first map and initial pose values.
    ASSERT_TRUE(wait_for_initialization());
    const auto [pose, _] = ndt_amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
  }

  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  const auto [estimate, _] = ndt_amcl_node_->estimate();

  {
    // Set new initial pose values that will be ignored.
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 1.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 29.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", -0.4});
  }

  spin_for(50ms, tester_node_, ndt_amcl_node_);

  {
    // Initializing with the second map but keeping the old estimate.
    // Ignoring the new initial pose values.
    const auto [pose, _] = ndt_amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), estimate.translation().x(), 0.01);
    ASSERT_NEAR(pose.translation().y(), estimate.translation().y(), 0.01);
    ASSERT_NEAR(pose.so2().log(), estimate.so2().log(), 0.01);
  }
}

TEST_F(TestNode, KeepCurrentEstimateAfterCleanup) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});

  {
    // Set initial pose values to simulate an estimate that has converged.
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  }

  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ndt_amcl_node_->deactivate();
  ndt_amcl_node_->cleanup();

  {
    // Set new initial pose values that will be ignored.
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 12.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 1.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.7});
  }

  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_NE(ndt_amcl_node_->particle_filter().get(), nullptr);
  ASSERT_TRUE(wait_for_initialization());
  const auto [pose, _] = ndt_amcl_node_->estimate();
  ASSERT_NEAR(pose.translation().x(), 12.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 1.0, 0.01);
  ASSERT_NEAR(pose.so2().log(), 0.7, 0.01);
}

TEST_F(TestNode, InvalidMotionModel) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"robot_model_type", "non_existing_model"});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_FALSE(wait_for_initialization());
}
TEST_F(TestNode, InvalidExecutionPolicy) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"execution_policy", "non_existing_policy"});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_FALSE(wait_for_initialization());
}

TEST_F(TestNode, SequentialExecutionPolicy) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"execution_policy", "seq"});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
}

TEST_F(TestNode, ParallelExecutionPolicy) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"execution_policy", "par"});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
}

TEST_F(TestNode, InitialPoseBeforeInitialize) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_FALSE(wait_for_pose_estimate());
  tester_node_->publish_default_initial_pose();
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
}

TEST_F(TestNode, InitialPoseAfterInitialize) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_default_initial_pose();
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, InitialPoseWithWrongFrame) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_initial_pose_with_wrong_frame();
  tester_node_->publish_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, CanUpdatePoseEstimate) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_default_initial_pose();
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(tester_node_->can_transform("map", "odom"));

  {
    const auto [pose, _] = ndt_amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 0.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 0.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0., 0.01);
  }

  tester_node_->publish_initial_pose(1., 1.);
  tester_node_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(tester_node_->can_transform("map", "odom"));

  {
    const auto [pose, _] = ndt_amcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 1.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 1.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0., 0.01);
  }
}

TEST_F(TestNode, IsPublishingParticleCloud) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->create_particle_cloud_subscriber();
  ASSERT_TRUE(wait_for_particle_cloud());
}

TEST_F(TestNode, LaserScanWithNoOdomToBase) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->publish_laser_scan_with_no_odom_to_base();
  ASSERT_FALSE(wait_for_pose_estimate());
}

TEST_F(TestNode, TransformValue) {
  ndt_amcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});

  {
    // Set initial pose values to simulate an estimate that has converged.
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 1.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", Sophus::Constants<double>::pi() / 3});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
    ndt_amcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  }

  ndt_amcl_node_->configure();
  ndt_amcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->publish_laser_scan_with_odom_to_base(Sophus::SE2d{
      Sophus::SO2d{Sophus::Constants<double>::pi() / 3},
      Sophus::Vector2d{3., 4.},
  });
  ASSERT_TRUE(wait_for_pose_estimate());

  const auto [pose, _] = ndt_amcl_node_->estimate();
  ASSERT_NEAR(pose.translation().x(), 1.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
  ASSERT_NEAR(pose.so2().log(), Sophus::Constants<double>::pi() / 3, 0.01);

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
        rclcpp::Parameter("laser_max_range", -1.0),
        rclcpp::Parameter("laser_min_range", -1.0),
        rclcpp::Parameter("max_beams", 1)));

TEST_P(TestParameterValue, InvalidValue) {
  const auto options = rclcpp::NodeOptions().parameter_overrides({GetParam()});
  ASSERT_ANY_THROW(std::make_shared<beluga_amcl::NdtAmclNode>(options));
}

}  // namespace

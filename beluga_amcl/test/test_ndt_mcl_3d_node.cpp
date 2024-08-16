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

#include <chrono>
#include <cstdlib>
#include <memory>
#include <sophus/so3.hpp>
#include <string>
#include <variant>

#include <sophus/common.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/utilities.hpp>

#include "beluga_amcl/ndt_mcl_node_3d.hpp"
#include "test_utils/node_testing.hpp"

namespace {

const std::string kTestMapPath = "./test_data/sample_3d_ndt_map.hdf5";

using namespace std::chrono_literals;
using beluga_amcl::testing::spin_for;
using beluga_amcl::testing::spin_until;

/// Test class that provides convenient public accessors.
class AmclNodeUnderTest : public beluga_amcl::NdtMclNode3D {
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
    ndt_mcl_node_ = std::make_shared<AmclNodeUnderTest>();
    tester_node_ = std::make_shared<beluga_amcl::testing::TesterNode>();
    tester_node_->create_transform_buffer();
    ndt_mcl_node_->set_parameter(rclcpp::Parameter("map_path", kTestMapPath));
    ndt_mcl_node_->set_parameter(rclcpp::Parameter("scan_topic", "point_cloud"));
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"num_particles", 10});
  }

  void TearDown() override { rclcpp::shutdown(); }

  bool wait_for_initialization() {
    return spin_until([this] { return ndt_mcl_node_->is_initialized(); }, 200ms, ndt_mcl_node_, tester_node_);
  }

  bool wait_for_pose_estimate() {
    tester_node_->latest_pose().reset();
    tester_node_->create_pose_subscriber();
    return spin_until([this] { return tester_node_->latest_pose().has_value(); }, 1000ms, ndt_mcl_node_, tester_node_);
  }

  bool wait_for_transform(const std::string& target, const std::string& source) {
    return spin_until(
        [&, this] { return tester_node_->can_transform(target, source); }, 1000ms, ndt_mcl_node_, tester_node_);
  }

  bool wait_for_particle_cloud() {
    tester_node_->create_particle_cloud_subscriber();
    return spin_until(
        [this] { return tester_node_->latest_particle_cloud().has_value(); }, 1000ms, ndt_mcl_node_, tester_node_);
  }

 protected:
  std::shared_ptr<AmclNodeUnderTest> ndt_mcl_node_;
  std::shared_ptr<beluga_amcl::testing::TesterNode> tester_node_;
};

class TestLifecycle : public ::testing::Test {
 public:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<beluga_amcl::NdtMclNode3D>();
    node_->set_parameter(rclcpp::Parameter("map_path", kTestMapPath));
    node_->set_parameter(rclcpp::Parameter("scan_topic", "point_cloud"));
    node_->set_parameter(rclcpp::Parameter{"num_particles", 10});
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<beluga_amcl::NdtMclNode3D> node_ = nullptr;
};

TEST_F(TestLifecycle, FullSpin) {
  using lifecycle_msgs::msg::State;
  ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(40ms, node_);
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(40ms, node_);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_for(40ms, node_);
  ASSERT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(40ms, node_);
  ASSERT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(40ms, node_);
  ASSERT_EQ(node_->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, ShutdownWhenActive) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtMclNode3D>();
  ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node_);
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node_);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_for(10ms, node_);
  ASSERT_EQ(node_->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, ShutdownWhenInactive) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtMclNode3D>();
  ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node_);
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node_);
  ASSERT_EQ(node_->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, ShutdownWhenUnconfigured) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtMclNode3D>();
  ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node_);
  ASSERT_EQ(node_->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, DestroyWhenActive) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtMclNode3D>();
  ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node_);
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node_);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TestLifecycle, DestroyWhenInactive) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtMclNode3D>();
  ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node_);
  ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(TestLifecycle, DestroyWhenUnconfigured) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtMclNode3D>();
  ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(TestLifecycle, AutoStart) {
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::NdtMclNode3D>(rclcpp::NodeOptions{}
                                                              .append_parameter_override("autostart", true)
                                                              .append_parameter_override("map_path", kTestMapPath)
                                                              .append_parameter_override("scan_topic", "point_cloud")
                                                              .append_parameter_override("num_particles", 10));
  spin_until([&] { return node->get_current_state().id() == State::PRIMARY_STATE_ACTIVE; }, 100ms, node);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

class TestInitializationWithModel : public BaseNodeFixture<::testing::TestWithParam<const char*>> {};

INSTANTIATE_TEST_SUITE_P(Models, TestInitializationWithModel, testing::Values("differential_drive"));

TEST_P(TestInitializationWithModel, ParticleCount) {
  const auto* const motion_model = GetParam();
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"robot_model_type", motion_model});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"scan_topic", "point_cloud"});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"num_particles", 10});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();

  ASSERT_TRUE(wait_for_initialization());

  std::visit([](const auto& pf) { ASSERT_EQ(pf.particles().size(), 10UL); }, *ndt_mcl_node_->particle_filter());
}

class TestNode : public BaseNodeFixture<::testing::Test> {};

TEST_F(TestNode, SetInitialPose) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.z", 2.0});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.roll", 0.2});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.pitch", 0.3});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.1});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  const auto [pose, _] = ndt_mcl_node_->estimate();
  ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);

  const auto expected_rotation = Sophus::SO3d::rotZ(0.1) * Sophus::SO3d::rotY(0.3) * Sophus::SO3d::rotX(0.2);
  ASSERT_TRUE(pose.so3().matrix().isApprox(expected_rotation.matrix(), 1e-6));
}

TEST_F(TestNode, BroadcastWhenInitialPoseSet) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_3d_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(wait_for_transform("map", "odom"));
}

TEST_F(TestNode, NoBroadcastWhenNoInitialPose) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  ASSERT_FALSE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, NoBroadcastWhenInitialPoseInvalid) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.0});
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", -0.5});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_3d_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, KeepCurrentEstimate) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  {
    // Set initial pose values to simulate an estimate that has converged.
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  }

  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();

  {
    // Initializing with the first map and initial pose values.
    ASSERT_TRUE(wait_for_initialization());
    const auto [pose, _] = ndt_mcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.translation().z(), 0., 0.01);
    ASSERT_NEAR(pose.so3().unit_quaternion().x(), 0., 0.01);
    ASSERT_NEAR(pose.so3().unit_quaternion().y(), 0., 0.01);
    ASSERT_NEAR(pose.so3().unit_quaternion().z(), 0., 0.01);
    ASSERT_NEAR(pose.so3().unit_quaternion().w(), 1., 0.01);
  }

  tester_node_->publish_3d_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  const auto [estimate, _] = ndt_mcl_node_->estimate();

  {
    // Set new initial pose values that will be ignored.
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 1.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 29.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", -0.4});
  }

  spin_for(50ms, tester_node_, ndt_mcl_node_);

  {
    // Initializing with the second map but keeping the old estimate.
    // Ignoring the new initial pose values.
    const auto [pose, _] = ndt_mcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), estimate.translation().x(), 0.01);
    ASSERT_NEAR(pose.translation().y(), estimate.translation().y(), 0.01);
    // ASSERT_NEAR(pose.so2().log(), estimate.so2().log(), 0.01);
  }
}

TEST_F(TestNode, KeepCurrentEstimateAfterCleanup) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});

  {
    // Set initial pose values to simulate an estimate that has converged.
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  }

  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->publish_3d_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ndt_mcl_node_->deactivate();
  ndt_mcl_node_->cleanup();

  {
    // Set new initial pose values that will be ignored.
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 12.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 1.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.7});
  }

  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_NE(ndt_mcl_node_->particle_filter().get(), nullptr);
  ASSERT_TRUE(wait_for_initialization());
  const auto [pose, _] = ndt_mcl_node_->estimate();
  ASSERT_NEAR(pose.translation().x(), 12.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 1.0, 0.01);
}

TEST_F(TestNode, InvalidMotionModel) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"robot_model_type", "non_existing_model"});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_FALSE(wait_for_initialization());
}
TEST_F(TestNode, InvalidExecutionPolicy) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"execution_policy", "non_existing_policy"});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_FALSE(wait_for_initialization());
}

TEST_F(TestNode, SequentialExecutionPolicy) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"execution_policy", "seq"});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
}

TEST_F(TestNode, ParallelExecutionPolicy) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"execution_policy", "par"});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
}

TEST_F(TestNode, InitialPoseBeforeInitialize) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_FALSE(wait_for_pose_estimate());
  tester_node_->publish_default_initial_pose();
  spin_for(10ms, ndt_mcl_node_);  // ensure orderly processing
  tester_node_->publish_3d_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
}

TEST_F(TestNode, InitialPoseAfterInitialize) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_default_initial_pose();
  spin_for(10ms, ndt_mcl_node_);  // ensure orderly processing
  tester_node_->publish_3d_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(wait_for_transform("map", "odom"));
}

TEST_F(TestNode, InitialPoseWithWrongFrame) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_initial_pose_with_wrong_frame();
  spin_for(10ms, ndt_mcl_node_);  // ensure orderly processing
  tester_node_->publish_3d_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
}

TEST_F(TestNode, CanUpdatePoseEstimate) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node_->can_transform("map", "odom"));
  tester_node_->publish_default_initial_pose();
  spin_for(10ms, ndt_mcl_node_);  // ensure orderly processing
  tester_node_->publish_3d_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(wait_for_transform("map", "odom"));

  {
    const auto [pose, _] = ndt_mcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 0.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 0.0, 0.01);
  }

  tester_node_->publish_initial_pose(1., 1.);
  spin_for(10ms, ndt_mcl_node_);  // ensure orderly processing
  tester_node_->publish_3d_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(wait_for_transform("map", "odom"));

  {
    const auto [pose, _] = ndt_mcl_node_->estimate();
    ASSERT_NEAR(pose.translation().x(), 1.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 1.0, 0.01);
  }
}

TEST_F(TestNode, IsPublishingParticleCloud) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->create_particle_cloud_subscriber();
  ASSERT_TRUE(wait_for_particle_cloud());
}

TEST_F(TestNode, LaserScanWithNoOdomToBase) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->publish_laser_scan_with_no_odom_to_base();
  ASSERT_FALSE(wait_for_pose_estimate());
}

TEST_F(TestNode, TransformValue) {
  ndt_mcl_node_->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  {
    // Set initial pose values to simulate an estimate that has converged.
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.x", 1.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.z", 3.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.roll", 0.});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.pitch", 0.});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.yaw", Sophus::Constants<double>::pi() / 3});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
    ndt_mcl_node_->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  }

  ndt_mcl_node_->configure();
  ndt_mcl_node_->activate();
  ASSERT_TRUE(wait_for_initialization());
  tester_node_->publish_3d_laser_scan_with_odom_to_base(Sophus::SE3d{
      Sophus::SO3d::rotZ(Sophus::Constants<double>::pi() / 3.),
      Sophus::Vector3d{3., 4., 5.},
  });
  ASSERT_TRUE(wait_for_pose_estimate());

  const auto [pose, _] = ndt_mcl_node_->estimate();
  ASSERT_NEAR(pose.translation().x(), 1.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
  // This is 0 cause we flatten the states as part of the motion model update process.
  ASSERT_NEAR(pose.translation().z(), 0.0, 0.01);
  ASSERT_NEAR(pose.so3().angleX(), 0., 0.01);
  ASSERT_NEAR(pose.so3().angleY(), 0., 0.01);
  ASSERT_NEAR(pose.so3().angleZ(), Sophus::Constants<double>::pi() / 3, 0.01);

  ASSERT_TRUE(wait_for_transform("map", "odom"));
  const auto transform = tester_node_->lookup_transform_3d("map", "odom");
  EXPECT_NEAR(transform.translation().x(), -2.00, 0.01);
  EXPECT_NEAR(transform.translation().y(), -2.00, 0.01);
  EXPECT_NEAR(transform.translation().z(), -5.00, 0.01);
  ASSERT_NEAR(pose.so3().angleX(), 0., 0.01);
  ASSERT_NEAR(pose.so3().angleY(), 0., 0.01);
  EXPECT_NEAR(transform.so3().angleZ(), 0.0, 0.01);
}

class TestParameterValue : public ::testing::TestWithParam<rclcpp::Parameter> {};

INSTANTIATE_TEST_SUITE_P(
    InvalidParameterValues,
    TestParameterValue,
    testing::Values(
        rclcpp::Parameter("num_particles", -5),
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
  const auto options = rclcpp::NodeOptions{}
                           .parameter_overrides({GetParam()})
                           .append_parameter_override("scan_topic", "point_cloud")
                           .append_parameter_override("num_particles", 10);
  ASSERT_ANY_THROW(std::make_shared<beluga_amcl::NdtMclNode3D>(options));
}

}  // namespace

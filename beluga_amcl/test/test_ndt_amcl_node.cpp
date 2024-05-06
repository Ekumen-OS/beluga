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

#include <gmock/gmock.h>
#include <tf2_ros/create_timer_ros.h>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <beluga_amcl/ndt_amcl_node.hpp>
#include <beluga_ros/tf2_sophus.hpp>

#include <cstdlib>
#include <iostream>
#include <lifecycle_msgs/msg/state.hpp>

namespace {

const std::string kTestMapPath = "./test_data/turtlebot3_world.hdf5";

using namespace std::chrono_literals;

/// Spin a group of nodes until a condition is met.
/**
 * \param predicate The stop condition.
 * \param timeout Maximum time to spin.
 * \param nodes The nodes to spin.
 * \return True if the condition was met. False if it timed out.
 */
template <class Predicate, class Rep, class Period, class... Nodes>
bool spin_until(
    Predicate&& predicate,
    const std::chrono::duration<Rep, Period>& timeout,
    const std::shared_ptr<Nodes>&... nodes) {
  rclcpp::executors::SingleThreadedExecutor executor;
  (executor.add_node(nodes->get_node_base_interface()), ...);

  const auto deadline = std::chrono::high_resolution_clock::now() + timeout;
  while (rclcpp::ok() && !predicate() && std::chrono::high_resolution_clock::now() < deadline) {
    executor.spin_once(deadline - std::chrono::high_resolution_clock::now());  // wait for it
    executor.spin_some(deadline - std::chrono::high_resolution_clock::now());  // flush it all out
  }
  return predicate();  // last minute check
}

/// Spin a group of nodes until a condition is met with a default timeout.
/**
 * \param predicate The stop condition.
 * \param nodes The nodes to spin.
 * \return True if the condition was met. False if it timed out.
 */
template <class Predicate, class... Nodes>
bool spin_until(Predicate&& predicate, const std::shared_ptr<Nodes>&... nodes) {
  return spin_until(std::forward<Predicate>(predicate), 10s, nodes...);
}

/// Spin a group of nodes for a given duration of time.
/**
 * \param predicate The stop condition.
 * \param duration Time to spin.
 */
template <class Rep, class Period, class... Nodes>
void spin_for(const std::chrono::duration<Rep, Period>& duration, const std::shared_ptr<Nodes>&... nodes) {
  const auto duration_is_over = []() { return false; };
  spin_until(duration_is_over, duration, nodes...);
}

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

// Tester node that can publish default messages and test ROS interactions.
class TesterNode : public rclcpp::Node {
 public:
  TesterNode() : rclcpp::Node{"tester_node", "", rclcpp::NodeOptions()} {
    initial_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::SystemDefaultsQoS());

    laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SystemDefaultsQoS());
  }

  void create_transform_buffer() {
    // NOTE(nahuel): This cannot be called in the constructor as it uses shared_from_this().
    // That's why we provide an initialization method.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface()));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
        *tf_buffer_, this,
        false);  // avoid using dedicated tf thread
  }

  void create_pose_subscriber() {
    pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "pose", rclcpp::SystemDefaultsQoS(), std::bind(&TesterNode::pose_callback, this, std::placeholders::_1));
  }

  void pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) { latest_pose_ = *message; }

  auto& latest_pose() { return latest_pose_; }

  void create_particle_cloud_subscriber() {
    particle_cloud_subscriber_ = create_subscription<nav2_msgs::msg::ParticleCloud>(
        "particle_cloud", rclcpp::SystemDefaultsQoS(),
        std::bind(&TesterNode::particle_cloud_callback, this, std::placeholders::_1));
  }

  void particle_cloud_callback(nav2_msgs::msg::ParticleCloud::SharedPtr message) { latest_particle_cloud_ = *message; }

  const auto& latest_particle_cloud() const { return latest_particle_cloud_; }

  static auto make_dummy_map() {
    auto map = nav_msgs::msg::OccupancyGrid{};
    map.header.frame_id = "map";
    map.info.resolution = 1.0;
    map.info.width = 2;
    map.info.height = 2;
    map.data = std::vector<std::int8_t>{0, 0, 0, 0};
    return map;
  }

  void publish_default_initial_pose() {
    auto pose = geometry_msgs::msg::PoseWithCovarianceStamped{};
    pose.header.frame_id = "map";
    initial_pose_publisher_->publish(pose);
  }

  void publish_initial_pose(double x, double y) {
    auto pose = geometry_msgs::msg::PoseWithCovarianceStamped{};
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    initial_pose_publisher_->publish(pose);
  }

  void publish_initial_pose_with_wrong_frame() {
    auto pose = geometry_msgs::msg::PoseWithCovarianceStamped{};
    pose.header.frame_id = "non_existing_frame";
    initial_pose_publisher_->publish(pose);
  }

  void publish_laser_scan() {
    const auto timestamp = now();

    auto scan = sensor_msgs::msg::LaserScan{};
    scan.header.stamp = timestamp;
    scan.header.frame_id = "laser";

    auto transform_base = geometry_msgs::msg::TransformStamped{};
    transform_base.header.stamp = timestamp;
    transform_base.header.frame_id = "odom";
    transform_base.child_frame_id = "base_footprint";

    auto transform_laser = geometry_msgs::msg::TransformStamped{};
    transform_laser.header.stamp = timestamp;
    transform_laser.header.frame_id = "base_footprint";
    transform_laser.child_frame_id = "laser";

    laser_scan_publisher_->publish(scan);
    tf_broadcaster_->sendTransform(transform_base);
    tf_broadcaster_->sendTransform(transform_laser);
  }

  void publish_laser_scan_with_no_odom_to_base() {
    const auto timestamp = now();

    auto scan = sensor_msgs::msg::LaserScan{};
    scan.header.stamp = timestamp;
    scan.header.frame_id = "laser";

    auto transform_base = geometry_msgs::msg::TransformStamped{};
    transform_base.header.stamp = timestamp;
    transform_base.header.frame_id = "odom";
    transform_base.child_frame_id = "unexpected_base";

    auto transform_laser = geometry_msgs::msg::TransformStamped{};
    transform_laser.header.stamp = timestamp;
    transform_laser.header.frame_id = "unexpected_base";
    transform_laser.child_frame_id = "laser";

    laser_scan_publisher_->publish(scan);
    tf_broadcaster_->sendTransform(transform_base);
    tf_broadcaster_->sendTransform(transform_laser);
  }

  void publish_laser_scan_with_odom_to_base(const Sophus::SE2d& transform) {
    const auto timestamp = now();

    auto scan = sensor_msgs::msg::LaserScan{};
    scan.header.stamp = timestamp;
    scan.header.frame_id = "laser";

    auto transform_base = geometry_msgs::msg::TransformStamped{};
    transform_base.header.stamp = timestamp;
    transform_base.header.frame_id = "odom";
    transform_base.child_frame_id = "base_footprint";
    transform_base.transform = tf2::toMsg(transform);

    auto transform_laser = geometry_msgs::msg::TransformStamped{};
    transform_laser.header.stamp = timestamp;
    transform_laser.header.frame_id = "base_footprint";
    transform_laser.child_frame_id = "laser";

    laser_scan_publisher_->publish(scan);
    tf_broadcaster_->sendTransform(transform_base);
    tf_broadcaster_->sendTransform(transform_laser);
  }

  bool can_transform(const std::string& target, const std::string& source) const {
    return tf_buffer_ && tf_buffer_->canTransform(target, source, tf2::TimePointZero);
  }

  auto lookup_transform(const std::string& target, const std::string& source) const {
    auto transform = Sophus::SE2d{};
    if (tf_buffer_) {
      tf2::convert(tf_buffer_->lookupTransform(target, source, tf2::TimePointZero).transform, transform);
    }
    return transform;
  }

 private:
  template <class Message>
  using PublisherPtr = std::shared_ptr<rclcpp::Publisher<Message>>;

  PublisherPtr<geometry_msgs::msg::PoseWithCovarianceStamped> initial_pose_publisher_;
  PublisherPtr<sensor_msgs::msg::LaserScan> laser_scan_publisher_;

  template <class Message>
  using SubscriberPtr = std::shared_ptr<rclcpp::Subscription<Message>>;

  SubscriberPtr<geometry_msgs::msg::PoseWithCovarianceStamped> pose_subscriber_;
  SubscriberPtr<nav2_msgs::msg::ParticleCloud> particle_cloud_subscriber_;

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> latest_pose_;
  std::optional<nav2_msgs::msg::ParticleCloud> latest_particle_cloud_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

/// Base node fixture class with common utilities.
template <class T>
class BaseNodeFixture : public T {
 public:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    ndt_amcl_node_ = std::make_shared<AmclNodeUnderTest>();
    tester_node_ = std::make_shared<TesterNode>();
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
  std::shared_ptr<TesterNode> tester_node_;
};

class TestLifecycle : public BaseNodeFixture<::testing::Test> {};

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

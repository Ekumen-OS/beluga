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

#include <beluga_amcl/private/amcl_node.hpp>

#include <lifecycle_msgs/msg/state.hpp>

namespace
{

using namespace std::chrono_literals;

/// Spin a group of nodes until a condition is met.
/**
 * \param predicate The stop condition.
 * \param timeout Maximum time to spin.
 * \param nodes The nodes to spin.
 * \return True if the condition was met. False if it timed out.
 */
template<class Predicate, class Rep, class Period, class ... Nodes>
bool spin_until(
  Predicate && predicate,
  const std::chrono::duration<Rep, Period> & timeout,
  const std::shared_ptr<Nodes> & ... nodes)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  (executor.add_node(nodes->get_node_base_interface()), ...);

  auto start = std::chrono::high_resolution_clock::now();
  while (std::chrono::high_resolution_clock::now() - start < timeout) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
  }
  return false;  // timed out
}

/// Spin a group of nodes until a condition is met with a default timeout.
/**
 * \param predicate The stop condition.
 * \param nodes The nodes to spin.
 * \return True if the condition was met. False if it timed out.
 */
template<class Predicate, class ... Nodes>
bool spin_until(
  Predicate && predicate,
  const std::shared_ptr<Nodes> & ... nodes)
{
  return spin_until(std::forward<Predicate>(predicate), 10s, nodes ...);
}

/// Spin a group of nodes for a given duration of time.
/**
 * \param predicate The stop condition.
 * \param duration Time to spin.
 */
template<class Rep, class Period, class ... Nodes>
void spin_for(
  const std::chrono::duration<Rep, Period> & duration,
  const std::shared_ptr<Nodes> & ... nodes)
{
  spin_until([]() {return false;}, duration, nodes ...);
}

/// Test class that provides convenient public accessors.
class AmclNodeUnderTest : public beluga_amcl::AmclNode
{
public:
  /// Get particle fitler pointer.
  const auto & particle_filter()
  {
    return particle_filter_;
  }

  /// Return true if the particle filter has been initialized.
  bool is_initialized() const
  {
    return particle_filter_.get();
  }
};

// Tester node that can publish default messages and test ROS interactions.
class TesterNode : public rclcpp::Node
{
public:
  TesterNode()
  : rclcpp::Node{"tester_node", "", rclcpp::NodeOptions()}
  {
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::SystemDefaultsQoS());

    initial_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose", rclcpp::SystemDefaultsQoS());

    laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SystemDefaultsQoS());

    global_localization_client_ = create_client<std_srvs::srv::Empty>(
      "reinitialize_global_localization");
  }

  void create_transform_buffer()
  {
    // NOTE(nahuel): This cannot be called in the constructor as it uses shared_from_this().
    // That's why we provide an initialization method.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface()));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_,
      this,
      false);  // avoid using dedicated tf thread
  }

  void create_pose_subscriber()
  {
    pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", rclcpp::SystemDefaultsQoS(),
      std::bind(&TesterNode::pose_callback, this, std::placeholders::_1));
  }

  void pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    latest_pose_ = *message;
  }

  const auto & latest_pose() const
  {
    return latest_pose_;
  }

  void create_particle_cloud_subscriber()
  {
    particle_cloud_subscriber_ = create_subscription<nav2_msgs::msg::ParticleCloud>(
      "particle_cloud", rclcpp::SystemDefaultsQoS(),
      std::bind(&TesterNode::particle_cloud_callback, this, std::placeholders::_1));
  }

  void particle_cloud_callback(nav2_msgs::msg::ParticleCloud::SharedPtr message)
  {
    latest_particle_cloud_ = *message;
  }

  const auto & latest_particle_cloud() const
  {
    return latest_particle_cloud_;
  }

  auto make_dummy_map()
  {
    auto map = nav_msgs::msg::OccupancyGrid{};
    map.header.frame_id = "map";
    map.info.resolution = 1.0;
    map.info.width = 2;
    map.info.height = 2;
    map.data = std::vector<std::int8_t>{0, 0, 0, 0};
    return map;
  }

  void publish_map()
  {
    map_publisher_->publish(make_dummy_map());
  }

  void publish_map_with_wrong_frame()
  {
    auto map = make_dummy_map();
    map.header.frame_id = "non_existing_frame";
    map_publisher_->publish(map);
  }

  void publish_default_initial_pose()
  {
    auto pose = geometry_msgs::msg::PoseWithCovarianceStamped{};
    pose.header.frame_id = "map";
    initial_pose_publisher_->publish(pose);
  }

  void publish_initial_pose_with_wrong_frame()
  {
    auto pose = geometry_msgs::msg::PoseWithCovarianceStamped{};
    pose.header.frame_id = "non_existing_frame";
    initial_pose_publisher_->publish(pose);
  }

  void publish_laser_scan()
  {
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

  void publish_laser_scan_with_no_odom_to_base()
  {
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

  bool can_transform(const std::string & source, const std::string & target)
  {
    return tf_buffer_ && tf_buffer_->canTransform(source, target, tf2::TimePointZero);
  }

  template<class Rep, class Period>
  bool wait_for_global_localization_service(const std::chrono::duration<Rep, Period> & timeout)
  {
    return global_localization_client_->wait_for_service(timeout);
  }

  auto async_request_global_localization()
  {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    return global_localization_client_->async_send_request(request);
  }

  auto prune_pending_global_localization_requests()
  {
    global_localization_client_->prune_pending_requests();
  }

private:
  template<class Message>
  using PublisherPtr = std::shared_ptr<rclcpp::Publisher<Message>>;

  PublisherPtr<nav_msgs::msg::OccupancyGrid> map_publisher_;
  PublisherPtr<geometry_msgs::msg::PoseWithCovarianceStamped> initial_pose_publisher_;
  PublisherPtr<sensor_msgs::msg::LaserScan> laser_scan_publisher_;

  template<class Message>
  using SubscriberPtr = std::shared_ptr<rclcpp::Subscription<Message>>;

  SubscriberPtr<geometry_msgs::msg::PoseWithCovarianceStamped> pose_subscriber_;
  SubscriberPtr<nav2_msgs::msg::ParticleCloud> particle_cloud_subscriber_;

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> latest_pose_;
  std::optional<nav2_msgs::msg::ParticleCloud> latest_particle_cloud_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<rclcpp::Client<std_srvs::srv::Empty>> global_localization_client_;
};

/// Base node fixture class with common utilities.
template<class T>
class BaseNodeFixture : public T
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    amcl_node = std::make_shared<AmclNodeUnderTest>();
    tester_node = std::make_shared<TesterNode>();
    tester_node->create_transform_buffer();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  bool activate_amcl()
  {
    using lifecycle_msgs::msg::State;
    if (amcl_node->get_current_state().id() == State::PRIMARY_STATE_UNCONFIGURED) {
      if (amcl_node->configure().id() == State::PRIMARY_STATE_INACTIVE) {
        if (amcl_node->activate().id() == State::PRIMARY_STATE_ACTIVE) {
          return true;
        }
      }
    }
    return false;
  }

  bool wait_for_initialization()
  {
    return spin_until(
      [this] {return amcl_node->is_initialized();},
      200ms, amcl_node, tester_node
    );
  }

  bool wait_for_pose_estimate()
  {
    tester_node->create_pose_subscriber();
    return spin_until(
      [this] {return tester_node->latest_pose().has_value();},
      100ms, amcl_node, tester_node);
  }

  bool wait_for_particle_cloud()
  {
    tester_node->create_particle_cloud_subscriber();
    return spin_until(
      [this] {return tester_node->latest_particle_cloud().has_value();},
      1000ms, amcl_node, tester_node);
  }

  bool request_global_localization()
  {
    if (!tester_node->wait_for_global_localization_service(500ms)) {
      return false;
    }
    auto future = tester_node->async_request_global_localization();
    bool done = spin_until(
      [&] {return future.wait_for(0s) == std::future_status::ready;},
      500ms, amcl_node, tester_node);
    if (done) {
      return true;
    } else {
      tester_node->prune_pending_global_localization_requests();
      return false;
    }
  }

protected:
  std::shared_ptr<AmclNodeUnderTest> amcl_node;
  std::shared_ptr<TesterNode> tester_node;
};

class TestLifecycle : public BaseNodeFixture<::testing::Test> {};

TEST_F(TestLifecycle, FullSpin)
{
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

TEST_F(TestLifecycle, ShutdownWhenActive)
{
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

TEST_F(TestLifecycle, ShutdownWhenInactive)
{
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, ShutdownWhenUnconfigured)
{
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestLifecycle, DestroyWhenActive)
{
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(10ms, node);
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TestLifecycle, DestroyWhenInactive)
{
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  spin_for(10ms, node);
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(TestLifecycle, DestroyWhenUnconfigured)
{
  using lifecycle_msgs::msg::State;
  auto node = std::make_shared<beluga_amcl::AmclNode>();
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

class TestInitializationWithModel : public BaseNodeFixture<::testing::TestWithParam<
      std::tuple<const char *, const char *>>> {};

INSTANTIATE_TEST_SUITE_P(
  Models,
  TestInitializationWithModel,
  testing::Values(
    std::make_tuple("differential_drive", "likelihood_field"),
    std::make_tuple("omnidirectional_drive", "beam"),
    std::make_tuple("stationary", "likelihood_field")));

TEST_P(TestInitializationWithModel, ParticleCount)
{
  const auto [motion_model, sensor_model] = GetParam();

  amcl_node->set_parameter(rclcpp::Parameter{"robot_model_type", motion_model});
  amcl_node->set_parameter(rclcpp::Parameter{"laser_model_type", sensor_model});
  amcl_node->set_parameter(rclcpp::Parameter{"min_particles", 10});
  amcl_node->set_parameter(rclcpp::Parameter{"max_particles", 30});

  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_TRUE(wait_for_initialization());

  ASSERT_GE(amcl_node->particle_filter()->particle_count(), 10UL);
  ASSERT_LE(amcl_node->particle_filter()->particle_count(), 30UL);
}

class TestNode : public BaseNodeFixture<::testing::Test> {};

TEST_F(TestNode, MapWithWrongFrame)
{
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map_with_wrong_frame();
  ASSERT_TRUE(wait_for_initialization());
}

TEST_F(TestNode, SetInitialPose)
{
  amcl_node->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  const auto [pose, _] = amcl_node->particle_filter()->estimate();
  ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
  ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
}

TEST_F(TestNode, BroadcastWhenInitialPoseSet)
{
  amcl_node->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node->can_transform("map", "odom"));
  tester_node->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(tester_node->can_transform("map", "odom"));
}

TEST_F(TestNode, NoBroadcastWhenNoInitialPose)
{
  amcl_node->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node->can_transform("map", "odom"));
  tester_node->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node->can_transform("map", "odom"));
}

TEST_F(TestNode, BroadcastWithGlobalLocalization)
{
  amcl_node->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node->can_transform("map", "odom"));
  ASSERT_TRUE(request_global_localization());
  tester_node->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(tester_node->can_transform("map", "odom"));
}

TEST_F(TestNode, IgnoreGlobalLocalizationBeforeInitialize)
{
  amcl_node->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ASSERT_TRUE(activate_amcl());
  ASSERT_TRUE(request_global_localization());
  tester_node->publish_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
}

TEST_F(TestNode, NoBroadcastWhenInitialPoseInvalid)
{
  amcl_node->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", -50.0});
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node->can_transform("map", "odom"));
  tester_node->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node->can_transform("map", "odom"));
}

TEST_F(TestNode, FirstMapOnly)
{
  amcl_node->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node->set_parameter(rclcpp::Parameter{"always_reset_initial_pose", true});

  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});

  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});

  amcl_node->set_parameter(rclcpp::Parameter{"first_map_only", true});
  ASSERT_TRUE(activate_amcl());

  {
    // Initializing with the first map and initial pose values.
    tester_node->publish_map();
    ASSERT_TRUE(wait_for_initialization());
    const auto [pose, _] = amcl_node->particle_filter()->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
  }

  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.x", 1.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.y", 29.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.yaw", -0.4});

  {
    // Ignoring the new initial pose values (and map).
    tester_node->publish_map();
    spin_for(50ms, tester_node, amcl_node);
    const auto [pose, _] = amcl_node->particle_filter()->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
  }

  amcl_node->set_parameter(rclcpp::Parameter{"first_map_only", false});

  {
    // Using the new initial pose values (and map).
    tester_node->publish_map();
    spin_for(50ms, tester_node, amcl_node);
    const auto [pose, _] = amcl_node->particle_filter()->estimate();
    ASSERT_NEAR(pose.translation().x(), 1.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 29.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), -0.4, 0.01);
  }
}

TEST_F(TestNode, KeepCurrentEstimate)
{
  amcl_node->set_parameter(rclcpp::Parameter{"set_initial_pose", true});
  amcl_node->set_parameter(rclcpp::Parameter{"always_reset_initial_pose", false});
  amcl_node->set_parameter(rclcpp::Parameter{"first_map_only", false});

  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_x", 0.001});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_y", 0.001});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yaw", 0.001});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xy", 0.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_xyaw", 0.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.covariance_yyaw", 0.0});

  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.x", 34.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.y", 2.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.yaw", 0.3});

  ASSERT_TRUE(activate_amcl());

  {
    // Initializing with the first map and initial pose values.
    tester_node->publish_map();
    ASSERT_TRUE(wait_for_initialization());
    const auto [pose, _] = amcl_node->particle_filter()->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
  }

  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.x", 1.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.y", 29.0});
  amcl_node->set_parameter(rclcpp::Parameter{"initial_pose.yaw", -0.4});

  tester_node->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  const auto [estimate, _] = amcl_node->particle_filter()->estimate();

  {
    // Initializing with the second map but keeping the old estimate.
    // Ignoring the new initial pose values.
    tester_node->publish_map();
    spin_for(50ms, tester_node, amcl_node);
    const auto [pose, _] = amcl_node->particle_filter()->estimate();
    ASSERT_NEAR(pose.translation().x(), estimate.translation().x(), 0.01);
    ASSERT_NEAR(pose.translation().y(), estimate.translation().y(), 0.01);
    ASSERT_NEAR(pose.so2().log(), estimate.so2().log(), 0.01);
  }
}

TEST_F(TestNode, InvalidMotionModel)
{
  amcl_node->set_parameter(rclcpp::Parameter{"robot_model_type", "non_existing_model"});
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_FALSE(wait_for_initialization());
}

TEST_F(TestNode, InvalidSensorModel)
{
  amcl_node->set_parameter(rclcpp::Parameter{"laser_model_type", "non_existing_model"});
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_FALSE(wait_for_initialization());
}

TEST_F(TestNode, InitialPoseBeforeInitialize)
{
  amcl_node->set_parameter(rclcpp::Parameter{"set_initial_pose", false});
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_default_initial_pose();
  tester_node->publish_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
}

TEST_F(TestNode, InitialPoseAfterInitialize)
{
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node->can_transform("map", "odom"));
  tester_node->publish_default_initial_pose();
  tester_node->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(tester_node->can_transform("map", "odom"));
}

TEST_F(TestNode, InitialPoseWithWrongFrame)
{
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_node->can_transform("map", "odom"));
  tester_node->publish_initial_pose_with_wrong_frame();
  tester_node->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_node->can_transform("map", "odom"));
}

TEST_F(TestNode, IsPublishingParticleCloud)
{
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  tester_node->create_particle_cloud_subscriber();
  ASSERT_TRUE(wait_for_particle_cloud());
}

TEST_F(TestNode, LaserScanWithNoOdomToBase)
{
  ASSERT_TRUE(activate_amcl());
  tester_node->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  tester_node->publish_laser_scan_with_no_odom_to_base();
  ASSERT_FALSE(wait_for_pose_estimate());
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
    rclcpp::Parameter("z_max", -1.0),
    rclcpp::Parameter("z_max", 2.0),
    rclcpp::Parameter("z_short", -1.0),
    rclcpp::Parameter("z_short", 2.0),
    rclcpp::Parameter("lambda_short", -1.0),
    rclcpp::Parameter("sigma_hit", -1.0)));

TEST_P(TestParameterValue, InvalidValue)
{
  const auto options = rclcpp::NodeOptions().parameter_overrides({GetParam()});
  ASSERT_ANY_THROW(std::make_shared<beluga_amcl::AmclNode>(options));
}

TEST_F(TestNode, InvalidValueDoesNotThrow)
{
  const auto options = rclcpp::NodeOptions()
    .parameter_overrides({rclcpp::Parameter("execution_policy", "x")});
  ASSERT_NO_THROW(std::make_shared<beluga_amcl::AmclNode>(options));
}

}  // namespace

// Copyright 2023-2024 Ekumen, Inc.
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

#include <dynamic_reconfigure/client.h>
#include <gmock/gmock.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>

#include <beluga_amcl/amcl_nodelet.hpp>

namespace {

using namespace std::chrono_literals;

/// Spin until a condition is met.
/**
 * \param predicate The stop condition.
 * \param timeout Maximum time to spin.
 * \return True if the condition was met. False if it timed out.
 */
template <class Predicate, class Rep, class Period>
bool spin_until(Predicate&& predicate, const std::chrono::duration<Rep, Period>& timeout) {
  const ros::Time deadline = ros::Time::now() + ros::Duration(std::chrono::duration<double>(timeout).count());
  ros::Rate r(100);
  while (ros::ok() && !predicate() && ros::Time::now() < deadline) {
    r.sleep();
  }
  return predicate();  // last minute check
}

/// Spin for a given duration of time.
/**
 * \param predicate The stop condition.
 * \param duration Time to spin.
 */
template <class Rep, class Period>
void spin_for(const std::chrono::duration<Rep, Period>& duration) {
  const auto duration_is_over = []() { return false; };
  spin_until(duration_is_over, duration);
}

/// Test class that provides convenient public accessors.
class AmclNodeletUnderTest : public beluga_amcl::AmclNodelet {
 public:
  /// Get particle filter pointer.
  const auto& particle_filter() { return particle_filter_; }

  /// Return true if the particle filter has been initialized.
  bool is_initialized() const { return particle_filter_ != nullptr; }

  /// Return the last known estimate. Throws if there is no estimate.
  const auto& estimate() { return last_known_estimate_.value(); }

  /// Retrieve nodelet default configuration (may fail).
  bool default_config(beluga_amcl::AmclConfig& config) {
    if (!config_client_) {
      config_client_ = std::make_unique<AmclConfigClient>(getName() + "/config_client", getPrivateNodeHandle());
    }
    return config_client_->getDefaultConfiguration(config);
  }

  /// Retrieve nodelet configuration (may fail).
  bool get(beluga_amcl::AmclConfig& config) {
    if (!config_client_) {
      config_client_ = std::make_unique<AmclConfigClient>(getName() + "/config_client", getPrivateNodeHandle());
    }
    return config_client_->getCurrentConfiguration(config);
  }

  /// Set nodelet configuration (may fail).
  bool set(beluga_amcl::AmclConfig& config) { return config_client_->setConfiguration(config); }

 private:
  using AmclConfigClient = dynamic_reconfigure::Client<beluga_amcl::AmclConfig>;
  std::unique_ptr<AmclConfigClient> config_client_;
};

// Tester node that can publish default messages and test ROS interactions.
class Tester {
 public:
  Tester() : tf_listener_(tf_buffer_) {
    map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);

    map_server_ = nh_.advertiseService("static_map", &Tester::static_map_callback);

    set_map_client_ = nh_.serviceClient<nav_msgs::SetMap>("set_map");

    initial_pose_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    laser_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);

    global_localization_client_ = nh_.serviceClient<std_srvs::Empty>("global_localization");
  }

  void create_pose_subscriber() {
    pose_subscriber_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "amcl_pose", 10, boost::bind(&Tester::pose_callback, this, _1));
  }

  void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message) { latest_pose_ = *message; }

  const auto& latest_pose() const { return latest_pose_; }

  void create_particle_cloud_subscriber() {
    particle_cloud_subscriber_ = nh_.subscribe<geometry_msgs::PoseArray>(
        "particlecloud", 10, boost::bind(&Tester::particle_cloud_callback, this, _1));
  }

  void particle_cloud_callback(const geometry_msgs::PoseArray::ConstPtr& message) { latest_particle_cloud_ = *message; }

  const auto& latest_particle_cloud() const { return latest_particle_cloud_; }

  static auto make_dummy_map() {
    auto map = nav_msgs::OccupancyGrid{};
    map.header.frame_id = "map";
    map.info.resolution = 1.0;
    map.info.width = 2;
    map.info.height = 2;
    map.data = std::vector<std::int8_t>{0, 0, 0, 0};
    return map;
  }

  void publish_map() { map_publisher_.publish(make_dummy_map()); }

  void publish_map_with_wrong_frame() {
    auto map = make_dummy_map();
    map.header.frame_id = "non_existing_frame";
    map_publisher_.publish(map);
  }

  bool set_map_and_initial_pose(double x, double y, double yaw) {
    nav_msgs::SetMap::Request request;
    nav_msgs::SetMap::Response response;
    request.map = make_dummy_map();
    request.initial_pose.header.frame_id = "map";
    request.initial_pose.pose.pose.position.x = x;
    request.initial_pose.pose.pose.position.y = y;
    request.initial_pose.pose.pose.orientation.x = 0.;
    request.initial_pose.pose.pose.orientation.y = 0.;
    request.initial_pose.pose.pose.orientation.z = std::sin(yaw / 2.);
    request.initial_pose.pose.pose.orientation.w = std::cos(yaw / 2.);
    return set_map_client_.call(request, response) && (response.success != 0U);
  }

  void publish_default_initial_pose() {
    auto pose = geometry_msgs::PoseWithCovarianceStamped{};
    pose.header.frame_id = "map";
    pose.pose.pose.orientation.w = 1.;
    initial_pose_publisher_.publish(pose);
  }

  void publish_initial_pose_with_wrong_frame() {
    auto pose = geometry_msgs::PoseWithCovarianceStamped{};
    pose.header.frame_id = "non_existing_frame";
    pose.pose.pose.orientation.w = 1.;
    initial_pose_publisher_.publish(pose);
  }

  void publish_laser_scan() {
    const auto timestamp = ros::Time::now();

    auto scan = sensor_msgs::LaserScan{};
    scan.header.stamp = timestamp;
    scan.header.frame_id = "laser";

    auto transform_base = geometry_msgs::TransformStamped{};
    transform_base.header.stamp = timestamp;
    transform_base.header.frame_id = "odom";
    transform_base.child_frame_id = "base_link";
    transform_base.transform.rotation.w = 1.;

    auto transform_laser = geometry_msgs::TransformStamped{};
    transform_laser.header.stamp = timestamp;
    transform_laser.header.frame_id = "base_link";
    transform_laser.child_frame_id = "laser";
    transform_laser.transform.rotation.w = 1.;

    tf_broadcaster_.sendTransform(transform_base);
    tf_broadcaster_.sendTransform(transform_laser);
    ros::Duration(0.2).sleep();  // mitigate race
    laser_scan_publisher_.publish(scan);
  }

  void publish_laser_scan_with_no_odom_to_base() {
    const auto timestamp = ros::Time::now();

    auto scan = sensor_msgs::LaserScan{};
    scan.header.stamp = timestamp;
    scan.header.frame_id = "laser";

    auto transform_base = geometry_msgs::TransformStamped{};
    transform_base.header.stamp = timestamp;
    transform_base.header.frame_id = "odom";
    transform_base.child_frame_id = "unexpected_base";
    transform_base.transform.rotation.w = 1.;

    auto transform_laser = geometry_msgs::TransformStamped{};
    transform_laser.header.stamp = timestamp;
    transform_laser.header.frame_id = "unexpected_base";
    transform_laser.child_frame_id = "laser";
    transform_laser.transform.rotation.w = 1.;

    tf_broadcaster_.sendTransform(transform_base);
    tf_broadcaster_.sendTransform(transform_laser);
    ros::Duration(0.2).sleep();  // mitigate race
    laser_scan_publisher_.publish(scan);
  }

  bool can_transform(const std::string& source, const std::string& target) const {
    return tf_buffer_.canTransform(source, target, ros::Time());
  }

  template <class Rep, class Period>
  bool wait_for_global_localization_service(const std::chrono::duration<Rep, Period>& timeout) {
    return global_localization_client_.waitForExistence(ros::Duration(std::chrono::duration<double>(timeout).count()));
  }

  bool request_global_localization() {
    std_srvs::Empty srv;
    return global_localization_client_.call(srv);
  }

 private:
  static bool static_map_callback(nav_msgs::GetMap::Request&, nav_msgs::GetMap::Response& response) {
    response.map = make_dummy_map();
    return true;
  }

  ros::NodeHandle nh_;

  ros::Publisher map_publisher_;
  ros::ServiceServer map_server_;
  ros::ServiceClient set_map_client_;

  ros::Publisher initial_pose_publisher_;
  ros::Publisher laser_scan_publisher_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber particle_cloud_subscriber_;

  std::optional<geometry_msgs::PoseWithCovarianceStamped> latest_pose_;
  std::optional<geometry_msgs::PoseArray> latest_particle_cloud_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::TransformListener tf_listener_;

  ros::ServiceClient global_localization_client_;
};

/// Base node fixture class with common utilities.
template <class T>
class BaseTestFixture : public T {
 public:
  void SetUp() override {
    amcl_nodelet_ = std::make_shared<AmclNodeletUnderTest>();
    amcl_nodelet_->init("amcl_nodelet", ros::names::getRemappings(), nodelet::V_string{}, nullptr, nullptr);
    tester_ = std::make_shared<Tester>();
  }

  void TearDown() override {}

  bool wait_for_initialization() {
    return spin_until([this] { return amcl_nodelet_->is_initialized(); }, 1000ms);
  }

  bool wait_for_pose_estimate() {
    return spin_until([this] { return tester_->latest_pose().has_value(); }, 1000ms);
  }

  bool wait_for_particle_cloud() {
    tester_->create_particle_cloud_subscriber();
    return spin_until([this] { return tester_->latest_particle_cloud().has_value(); }, 1000ms);
  }

  bool request_global_localization() {
    if (!tester_->wait_for_global_localization_service(500ms)) {
      return false;
    }
    return tester_->request_global_localization();
  }

 protected:
  std::shared_ptr<AmclNodeletUnderTest> amcl_nodelet_;
  std::shared_ptr<Tester> tester_;
};

class TestInitializationWithModel
    : public BaseTestFixture<::testing::TestWithParam<std::tuple<const char*, const char*>>> {};

INSTANTIATE_TEST_SUITE_P(
    Models,
    TestInitializationWithModel,
    testing::Values(
        std::make_tuple("differential_drive", "likelihood_field"),
        std::make_tuple("omnidirectional_drive", "beam"),
        std::make_tuple("stationary", "likelihood_field")));

TEST_P(TestInitializationWithModel, ParticleCount) {
  const auto& [motion_model, sensor_model] = GetParam();

  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.odom_model_type = motion_model;
  config.laser_model_type = sensor_model;
  config.min_particles = 10;
  config.max_particles = 30;
  ASSERT_TRUE(amcl_nodelet_->set(config));

  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());

  ASSERT_GE(amcl_nodelet_->particle_filter()->particles().size(), 10UL);
  ASSERT_LE(amcl_nodelet_->particle_filter()->particles().size(), 30UL);
}

class TestFixture : public BaseTestFixture<::testing::Test> {};

TEST_F(TestFixture, MapWithWrongFrame) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map_with_wrong_frame();
  ASSERT_TRUE(wait_for_initialization());
}

class MapFromServiceFixture : public TestFixture {
 public:
  void SetUp() override {
    nh_.setParam("use_map_topic", false);
    TestFixture::SetUp();
  }

  void TearDown() override { nh_.deleteParam("use_map_topic"); }

 private:
  ros::NodeHandle nh_{"/amcl_nodelet"};
};

TEST_F(MapFromServiceFixture, MapFromService) {
  ASSERT_TRUE(wait_for_initialization());
}

TEST_F(TestFixture, SetInitialPose) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = true;
  config.initial_pose_x = 34.0;
  config.initial_pose_y = 2.0;
  config.initial_pose_a = 0.3;
  config.initial_cov_xx = 0.001;
  config.initial_cov_yy = 0.001;
  config.initial_cov_aa = 0.001;
  config.initial_cov_xy = 0.0;
  config.initial_cov_xa = 0.0;
  config.initial_cov_ya = 0.0;
  ASSERT_TRUE(amcl_nodelet_->set(config));

  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  const auto [pose, _] = amcl_nodelet_->estimate();
  ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
  ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
  ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
}

TEST_F(TestFixture, SetMapAndInitialPose) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = true;
  config.initial_pose_x = -1.0;
  config.initial_pose_y = 1.0;
  config.initial_pose_a = 1.0;
  config.initial_cov_xx = 0.001;
  config.initial_cov_yy = 0.001;
  config.initial_cov_aa = 0.001;
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  {
    const auto [pose, _] = amcl_nodelet_->estimate();
    ASSERT_NEAR(pose.translation().x(), -1.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 1.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 1.0, 0.01);
  }
  ASSERT_TRUE(tester_->set_map_and_initial_pose(1.0, -1.0, -1.0));
  {
    const auto [pose, _] = amcl_nodelet_->estimate();
    ASSERT_NEAR(pose.translation().x(), 1.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), -1.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), -1.0, 0.01);
  }
}

TEST_F(TestFixture, BroadcastWhenInitialPoseSet) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = true;
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_->can_transform("map", "odom"));
  tester_->create_pose_subscriber();
  tester_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(tester_->can_transform("map", "odom"));
}

TEST_F(TestFixture, NoBroadcastWhenNoInitialPose) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = false;
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_->can_transform("map", "odom"));
  tester_->create_pose_subscriber();
  tester_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_->can_transform("map", "odom"));
}

TEST_F(TestFixture, BroadcastWithGlobalLocalization) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = false;
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_->can_transform("map", "odom"));
  ASSERT_TRUE(request_global_localization());
  tester_->create_pose_subscriber();
  tester_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(tester_->can_transform("map", "odom"));
}

TEST_F(TestFixture, IgnoreGlobalLocalizationBeforeInitialize) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = false;
  ASSERT_TRUE(amcl_nodelet_->set(config));
  ASSERT_FALSE(request_global_localization());
  tester_->create_pose_subscriber();
  tester_->publish_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
}

TEST_F(TestFixture, NoBroadcastWhenInitialPoseInvalid) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = true;
  config.initial_cov_xx = 0.0;
  config.initial_cov_yy = 0.0;
  config.initial_cov_xy = -50.0;
  ASSERT_TRUE(amcl_nodelet_->set(config));

  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_->can_transform("map", "odom"));
  tester_->create_pose_subscriber();
  tester_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_->can_transform("map", "odom"));
}

TEST_F(TestFixture, FirstMapOnly) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = true;
  config.always_reset_initial_pose = true;

  {
    // Set initial pose values to simulate an estimate that has converged.
    config.initial_pose_x = 34.0;
    config.initial_pose_y = 2.0;
    config.initial_pose_a = 0.3;
    config.initial_cov_xx = 0.001;
    config.initial_cov_yy = 0.001;
    config.initial_cov_aa = 0.001;
    config.initial_cov_xy = 0.0;
    config.initial_cov_xa = 0.0;
    config.initial_cov_ya = 0.0;
  }

  config.first_map_only = true;
  ASSERT_TRUE(amcl_nodelet_->set(config));

  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());

  {
    // Initialized with the first map and initial pose values.
    const auto [pose, _] = amcl_nodelet_->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
  }

  {
    // Set new initial pose values that will be ignored.
    config.initial_pose_x = 1.0;
    config.initial_pose_y = 29.0;
    config.initial_pose_a = -0.4;
    ASSERT_TRUE(amcl_nodelet_->set(config));
  }

  tester_->publish_map();
  spin_for(50ms);

  {
    // Ignored the new initial pose values (and map).
    const auto [pose, _] = amcl_nodelet_->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
  }

  config.first_map_only = false;
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  spin_for(50ms);

  {
    // Initialized with the new initial pose values (and map).
    const auto [pose, _] = amcl_nodelet_->estimate();
    ASSERT_NEAR(pose.translation().x(), 1.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 29.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), -0.4, 0.01);
  }
}

TEST_F(TestFixture, KeepCurrentEstimate) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = true;
  config.always_reset_initial_pose = false;
  config.first_map_only = false;

  {
    // Set initial pose values to simulate an estimate that has converged.
    config.initial_pose_x = 34.0;
    config.initial_pose_y = 2.0;
    config.initial_pose_a = 0.3;
    config.initial_cov_xx = 0.001;
    config.initial_cov_yy = 0.001;
    config.initial_cov_aa = 0.001;
    config.initial_cov_xy = 0.0;
    config.initial_cov_xa = 0.0;
    config.initial_cov_ya = 0.0;
  }
  ASSERT_TRUE(amcl_nodelet_->set(config));

  {
    // Initializing with the first map and initial pose values.
    tester_->publish_map();
    ASSERT_TRUE(wait_for_initialization());
    const auto [pose, _] = amcl_nodelet_->estimate();
    ASSERT_NEAR(pose.translation().x(), 34.0, 0.01);
    ASSERT_NEAR(pose.translation().y(), 2.0, 0.01);
    ASSERT_NEAR(pose.so2().log(), 0.3, 0.01);
  }

  tester_->create_pose_subscriber();
  tester_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  const auto [estimate, _] = amcl_nodelet_->estimate();

  {
    // Set new initial pose values that will be ignored.
    config.initial_pose_x = 1.0;
    config.initial_pose_y = 29.0;
    config.initial_pose_a = -0.4;
  }
  ASSERT_TRUE(amcl_nodelet_->set(config));

  tester_->publish_map();
  spin_for(50ms);

  {
    // Initializing with the second map but keeping the old estimate.
    // Ignoring the new initial pose values.
    const auto [pose, _] = amcl_nodelet_->estimate();
    ASSERT_NEAR(pose.translation().x(), estimate.translation().x(), 0.01);
    ASSERT_NEAR(pose.translation().y(), estimate.translation().y(), 0.01);
    ASSERT_NEAR(pose.so2().log(), estimate.so2().log(), 0.01);
  }
}

TEST_F(TestFixture, InvalidMotionModel) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.odom_model_type = "non_existing_model";
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  ASSERT_FALSE(wait_for_initialization());
}

TEST_F(TestFixture, InvalidSensorModel) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.laser_model_type = "non_existing_model";
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  ASSERT_FALSE(wait_for_initialization());
}

TEST_F(TestFixture, InitialPoseBeforeInitialize) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = false;
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_default_initial_pose();
  tester_->create_pose_subscriber();
  tester_->publish_laser_scan();
  ASSERT_FALSE(wait_for_pose_estimate());
}

TEST_F(TestFixture, InitialPoseAfterInitialize) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = false;
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_->can_transform("map", "odom"));
  tester_->publish_default_initial_pose();
  tester_->create_pose_subscriber();
  tester_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_TRUE(tester_->can_transform("map", "odom"));
}

TEST_F(TestFixture, InitialPoseWithWrongFrame) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = false;
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  ASSERT_FALSE(tester_->can_transform("map", "odom"));
  tester_->publish_initial_pose_with_wrong_frame();
  tester_->create_pose_subscriber();
  tester_->publish_laser_scan();
  ASSERT_TRUE(wait_for_pose_estimate());
  ASSERT_FALSE(tester_->can_transform("map", "odom"));
}

TEST_F(TestFixture, IsPublishingParticleCloud) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  tester_->create_particle_cloud_subscriber();
  ASSERT_TRUE(wait_for_particle_cloud());
}

TEST_F(TestFixture, LaserScanWithNoOdomToBase) {
  beluga_amcl::AmclConfig config;
  ASSERT_TRUE(amcl_nodelet_->default_config(config));
  config.set_initial_pose = true;
  ASSERT_TRUE(amcl_nodelet_->set(config));
  tester_->publish_map();
  ASSERT_TRUE(wait_for_initialization());
  tester_->create_pose_subscriber();
  tester_->publish_laser_scan_with_no_odom_to_base();
  ASSERT_FALSE(wait_for_pose_estimate());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_amcl_nodelet");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}

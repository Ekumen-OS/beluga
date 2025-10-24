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

#ifndef BELUGA_AMCL_TEST_TEST_UTILS_NODE_TESTING_HPP  // NOLINT(llvm-header-guard,-warnings-as-errors)
#define BELUGA_AMCL_TEST_TEST_UTILS_NODE_TESTING_HPP  // NOLINT(llvm-header-guard,-warnings-as-errors)

#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sophus/se2.hpp>

#include <bondcpp/bond.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/convert.hpp>
#include <tf2/utils.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <beluga_ros/tf2_sophus.hpp>

namespace beluga_amcl::testing {

using namespace std::chrono_literals;

// Tester node that can publish default messages and test ROS interactions.
class TesterNode : public rclcpp::Node {
 public:
  TesterNode() : rclcpp::Node{"tester_node", "", rclcpp::NodeOptions()} {
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::SystemDefaultsQoS());

    initial_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::SystemDefaultsQoS());

    laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SystemDefaultsQoS());

    point_cloud_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SystemDefaultsQoS());

    global_localization_client_ = create_client<std_srvs::srv::Empty>("reinitialize_global_localization");

    nomotion_update_client_ = create_client<std_srvs::srv::Empty>("request_nomotion_update");
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
    particle_cloud_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "particle_cloud", rclcpp::SystemDefaultsQoS(),
        std::bind(&TesterNode::particle_cloud_callback, this, std::placeholders::_1));
  }

  void particle_cloud_callback(geometry_msgs::msg::PoseArray::SharedPtr message) { latest_particle_cloud_ = *message; }

  const auto& latest_particle_cloud() const { return latest_particle_cloud_; }

  void create_particle_markers_subscriber() {
    particle_markers_subscriber_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "particle_markers", rclcpp::SystemDefaultsQoS(),
        std::bind(&TesterNode::particle_markers_callback, this, std::placeholders::_1));
  }

  void particle_markers_callback(visualization_msgs::msg::MarkerArray::SharedPtr message) {
    latest_particle_markers_ = *message;
  }

  const auto& latest_particle_markers() const { return latest_particle_markers_; }

  void create_likelihood_field_subscriber() {
    auto qos = rclcpp::SystemDefaultsQoS();
    qos.reliable().transient_local();
    likelihood_field_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "likelihood_field", qos, std::bind(&TesterNode::likelihood_field_callback, this, std::placeholders::_1));
  }

  void likelihood_field_callback(nav_msgs::msg::OccupancyGrid::SharedPtr message) {
    latest_likelihood_field_ = *message;
  }

  const auto& latest_likelihood_field() const { return latest_likelihood_field_; }

  static auto make_dummy_map() {
    auto map = nav_msgs::msg::OccupancyGrid{};
    map.header.frame_id = "map";
    map.info.resolution = 1.0;
    map.info.width = 2;
    map.info.height = 2;
    map.data = std::vector<std::int8_t>{0, 0, 0, 0};
    return map;
  }

  void publish_map() { map_publisher_->publish(make_dummy_map()); }

  void publish_map_with_wrong_frame() {
    auto map = make_dummy_map();
    map.header.frame_id = "non_existing_frame";
    map_publisher_->publish(map);
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

  void publish_3d_laser_scan() {
    const auto timestamp = now();

    auto scan = sensor_msgs::msg::PointCloud2{};
    scan.header.stamp = timestamp;
    scan.header.frame_id = "laser";
    // Modifier to describe what the fields are.
    sensor_msgs::PointCloud2Modifier modifier(scan);

    modifier.setPointCloud2Fields(
        3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32);

    auto transform_base = geometry_msgs::msg::TransformStamped{};
    transform_base.header.stamp = timestamp;
    transform_base.header.frame_id = "odom";
    transform_base.child_frame_id = "base_footprint";

    auto transform_laser = geometry_msgs::msg::TransformStamped{};
    transform_laser.header.stamp = timestamp;
    transform_laser.header.frame_id = "base_footprint";
    transform_laser.child_frame_id = "laser";

    point_cloud_publisher_->publish(scan);
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

  void publish_3d_laser_scan_with_no_odom_to_base() {
    const auto timestamp = now();
    auto scan = sensor_msgs::msg::PointCloud2{};

    // Modifier to describe what the fields are.
    sensor_msgs::PointCloud2Modifier modifier(scan);

    modifier.setPointCloud2Fields(
        3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32);

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

    point_cloud_publisher_->publish(scan);
    tf_broadcaster_->sendTransform(transform_base);
    tf_broadcaster_->sendTransform(transform_laser);
  }

  void publish_point_cloud() {
    const auto timestamp = now();

    auto cloud = sensor_msgs::msg::PointCloud2{};
    cloud.header.stamp = timestamp;
    cloud.header.frame_id = "laser";
    cloud.height = 1;
    cloud.width = 1;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(1);

    sensor_msgs::PointCloud2Iterator<double> iter_x(cloud, "x");  // NOLINT(misc-const-correctness)
    sensor_msgs::PointCloud2Iterator<double> iter_y(cloud, "y");  // NOLINT(misc-const-correctness)
    sensor_msgs::PointCloud2Iterator<double> iter_z(cloud, "z");  // NOLINT(misc-const-correctness)

    *iter_x = 1.0;
    *iter_y = 0.0;
    *iter_z = 0.0;

    auto transform_base = geometry_msgs::msg::TransformStamped{};
    transform_base.header.stamp = timestamp;
    transform_base.header.frame_id = "odom";
    transform_base.child_frame_id = "base_footprint";

    auto transform_laser = geometry_msgs::msg::TransformStamped{};
    transform_laser.header.stamp = timestamp;
    transform_laser.header.frame_id = "base_footprint";
    transform_laser.child_frame_id = "laser";

    point_cloud_publisher_->publish(cloud);
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

  void publish_3d_laser_scan_with_odom_to_base(const Sophus::SE3d& transform) {
    const auto timestamp = now();

    auto scan = sensor_msgs::msg::PointCloud2{};

    // Modifier to describe what the fields are.
    sensor_msgs::PointCloud2Modifier modifier(scan);

    modifier.setPointCloud2Fields(
        3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32);

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

    point_cloud_publisher_->publish(scan);
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

  auto lookup_transform_3d(const std::string& target, const std::string& source) const {
    auto transform = Sophus::SE3d{};
    if (tf_buffer_) {
      tf2::convert(tf_buffer_->lookupTransform(target, source, tf2::TimePointZero).transform, transform);
    }
    return transform;
  }

  template <class Rep, class Period>
  bool wait_for_global_localization_service(const std::chrono::duration<Rep, Period>& timeout) const {
    return global_localization_client_->wait_for_service(timeout);
  }

  auto async_request_global_localization() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    return global_localization_client_->async_send_request(request);
  }

  auto prune_pending_global_localization_requests() { global_localization_client_->prune_pending_requests(); }

  auto async_nomotion_update_request() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    return nomotion_update_client_->async_send_request(request);
  }

  auto prune_pending_nomotion_update_request() { nomotion_update_client_->prune_pending_requests(); }

 private:
  template <class Message>
  using PublisherPtr = std::shared_ptr<rclcpp::Publisher<Message>>;

  PublisherPtr<nav_msgs::msg::OccupancyGrid> map_publisher_;
  PublisherPtr<geometry_msgs::msg::PoseWithCovarianceStamped> initial_pose_publisher_;
  PublisherPtr<sensor_msgs::msg::LaserScan> laser_scan_publisher_;
  PublisherPtr<sensor_msgs::msg::PointCloud2> point_cloud_publisher_;

  template <class Message>
  using SubscriberPtr = std::shared_ptr<rclcpp::Subscription<Message>>;

  SubscriberPtr<geometry_msgs::msg::PoseWithCovarianceStamped> pose_subscriber_;
  SubscriberPtr<geometry_msgs::msg::PoseArray> particle_cloud_subscriber_;
  SubscriberPtr<visualization_msgs::msg::MarkerArray> particle_markers_subscriber_;
  SubscriberPtr<nav_msgs::msg::OccupancyGrid> likelihood_field_subscriber_;

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> latest_pose_;
  std::optional<geometry_msgs::msg::PoseArray> latest_particle_cloud_;
  std::optional<visualization_msgs::msg::MarkerArray> latest_particle_markers_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_likelihood_field_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<rclcpp::Client<std_srvs::srv::Empty>> global_localization_client_;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Empty>> nomotion_update_client_;
};

/// Spin a group of nodes until a condition is met.
/**
 * \param predicate The stop condition.
 * \param timeout Maximum time to spin.
 * \param nodes The nodes to spin.
 * \return True if the condition was met. False if it timed out.
 */
template <class Predicate, class Rep, class Period, class... Nodes>
inline bool spin_until(
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
inline bool spin_until(Predicate&& predicate, const std::shared_ptr<Nodes>&... nodes) {
  return spin_until(std::forward<Predicate>(predicate), 10s, nodes...);
}

/// Spin a group of nodes for a given duration of time.
/**
 * \param predicate The stop condition.
 * \param duration Time to spin.
 */
template <class Rep, class Period, class... Nodes>
inline void spin_for(const std::chrono::duration<Rep, Period>& duration, const std::shared_ptr<Nodes>&... nodes) {
  const auto duration_is_over = []() { return false; };
  spin_until(duration_is_over, duration, nodes...);
}

}  // namespace beluga_amcl::testing
#endif

// Copyright 2022-2024 Ekumen, Inc.
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

#ifndef BELUGA_AMCL_AMCL_NODE_HPP
#define BELUGA_AMCL_AMCL_NODE_HPP

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <utility>

#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include <beluga/beluga.hpp>
#include <beluga_ros/amcl.hpp>

/**
 * \file
 * \brief ROS 2 integration of the 2D AMCL algorithm.
 */

namespace beluga_amcl {

/// 2D AMCL as a ROS 2 composable lifecycle node.
class AmclNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// Constructor.
  explicit AmclNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AmclNode() override;

 protected:
  /// Callback for lifecycle transitions from the UNCONFIGURED state to the INACTIVE state.
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;

  /// Callback for lifecycle transitions from the INACTIVE state to the ACTIVE state.
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

  /// Callback for lifecycle transitions from the ACTIVE state to the INACTIVE state.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  /// Callback for lifecycle transitions from the INACTIVE state to the UNCONFIGURED state.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;

  /// Callback for lifecycle transitions from most states to the FINALIZED state.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

  /// Get initial pose estimate from parameters if set.
  auto get_initial_estimate() const -> std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>>;

  /// Get motion model as per current parametrization.
  auto get_motion_model(std::string_view) const -> beluga_ros::Amcl::motion_model_variant;

  /// Get sensor model as per current parametrization.
  auto get_sensor_model(std::string_view, nav_msgs::msg::OccupancyGrid::SharedPtr) const
      -> beluga_ros::Amcl::sensor_model_variant;

  /// Get execution policy given its name.
  static auto get_execution_policy(std::string_view) -> beluga_ros::Amcl::execution_policy_variant;

  /// Instantiate particle filter given an initial occupancy grid map and the current parametrization.
  auto make_particle_filter(nav_msgs::msg::OccupancyGrid::SharedPtr) const -> std::unique_ptr<beluga_ros::Amcl>;

  /// Callback for occupancy grid map updates.
  void map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr);

  /// Callback for periodic particle cloud updates.
  void timer_callback();

  /// Callback for node to configure and activate itself.
  void autostart_callback();

  /// Callback for laser scan updates.
  void laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr);

  /// Callback for pose (re)initialization.
  void initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);

  /// Callback for the global relocalization service.
  void global_localization_callback(
      std::shared_ptr<rmw_request_id_t> request_header,
      std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /// Callback for the no motion update service.
  void nomotion_update_callback(
      std::shared_ptr<rmw_request_id_t> request_header,
      std::shared_ptr<std_srvs::srv::Empty::Request> req,
      std::shared_ptr<std_srvs::srv::Empty::Response> res);

  /// Initialize particles from an estimated pose and covariance.
  /**
   * If an exception occurs during the initialization, an error message is logged, and the initialization
   * process is also aborted, returning false. If the initialization is successful, the TF broadcast is
   * enabled.
   *
   * \param estimate A pair representing the estimated pose and covariance for initialization.
   * \return True if the initialization is successful, false otherwise.
   */
  bool initialize_from_estimate(const std::pair<Sophus::SE2d, Eigen::Matrix3d>& estimate);

  /// Initialize particles from map.
  /**
   * If an exception occurs during the initialization, an error message is logged, and the initialization
   * process is also aborted, returning false. If the initialization is successful, the TF broadcast is
   * enabled.
   *
   * \return True if the initialization is successful, false otherwise.
   */
  bool initialize_from_map();

  /// Node bond with the lifecycle manager.
  std::unique_ptr<bond::Bond> bond_;

  /// Timer for periodic particle cloud updates.
  rclcpp::TimerBase::SharedPtr timer_;

  /// Timer for node to configure and activate itself.
  rclcpp::TimerBase::SharedPtr autostart_timer_;

  /// Particle cloud publisher.
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_cloud_pub_;
  /// Estimated pose publisher.
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  /// Pose (re)initialization subscription.
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  /// Occupancy grid map updates subscription.
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  /// Laser scan updates subscription.
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>>
      laser_scan_sub_;

  /// Global relocalization service server.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_localization_server_;
  /// No motion update service server.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_server_;

  /// Transforms buffer.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  /// Transforms broadcaster.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  /// Transforms listener.
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  /// Transform synchronization filter for laser scan updates.
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  /// Connection for laser scan updates filter and callback.
  message_filters::Connection laser_scan_connection_;

  /// Particle filter instance.
  std::unique_ptr<beluga_ros::Amcl> particle_filter_;
  /// Last known pose estimate, if any.
  std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> last_known_estimate_;
  /// Last known map to odom correction estimate, if any.
  std::optional<Sophus::SE2d> last_known_odom_transform_in_map_;
  /// Whether to broadcast transforms or not.
  bool enable_tf_broadcast_{false};
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL_AMCL_NODE_HPP

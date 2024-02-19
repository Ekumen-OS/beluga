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
#include <beluga_ros/amcl_impl.hpp>

namespace beluga_amcl {

class AmclNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit AmclNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AmclNode() override;

 protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

  auto get_initial_estimate() -> std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>>;

  auto get_motion_model(std::string_view) -> beluga_ros::AmclImpl::motion_model_variant;

  auto get_sensor_model(std::string_view, nav_msgs::msg::OccupancyGrid::SharedPtr)
      -> beluga_ros::AmclImpl::sensor_model_variant;

  auto get_execution_policy(std::string_view) -> beluga_ros::AmclImpl::execution_policy_variant;

  auto get_amcl_impl(nav_msgs::msg::OccupancyGrid::SharedPtr) -> std::unique_ptr<beluga_ros::AmclImpl>;

  void map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr);

  void timer_callback();

  void laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr);

  void initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);

  void global_localization_callback(
      std::shared_ptr<rmw_request_id_t> request_header,
      std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

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
   * The TF broadcast is not enabled during the initialization to match the original implementation,
   * as it is typically enabled only in response to external global localization requests and not during
   * the initial setup of the filter.
   *
   * \return True if the initialization is successful, false otherwise.
   */
  bool initialize_from_map();

  std::unique_ptr<bond::Bond> bond_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>>
      laser_scan_sub_;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_localization_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_server_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  message_filters::Connection laser_scan_connection_;

  std::unique_ptr<beluga_ros::AmclImpl> impl_;
  std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> last_known_estimate_;
  bool enable_tf_broadcast_{false};
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL_PRIVATE_AMCL_NODE_HPP

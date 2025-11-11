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

#ifndef BELUGA_AMCL_ROS2_COMMON_HPP
#define BELUGA_AMCL_ROS2_COMMON_HPP

#include <execution>
#include <rclcpp/node_options.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <string_view>

#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.hpp>
#include <tf2/utils.hpp>

#include <beluga_ros/tf2_sophus.hpp>

namespace beluga_amcl {

/// String identifier for a diff drive model.
constexpr std::string_view kDifferentialModelName = "differential_drive";
/// String identifier for a omnidirectional drive model.
constexpr std::string_view kOmnidirectionalModelName = "omnidirectional_drive";
/// String identifier for a stationary model.
constexpr std::string_view kStationaryModelName = "stationary";
/// String identifier for a diff drive model.
constexpr std::string_view kNav2DifferentialModelName = "nav2_amcl::DifferentialMotionModel";
/// String identifier for a omnidirectional model name.
constexpr std::string_view kNav2OmnidirectionalModelName = "nav2_amcl::OmniMotionModel";
/// String identifier for a ackermann drive model.
constexpr std::string_view kAckermannDriveModelName = "ackermann_drive";
/// Supported execution policies.
using ExecutionPolicyVariant = std::variant<std::execution::sequenced_policy, std::execution::parallel_policy>;

/// Base AMCL lifecycle node, with some basic common functionalities, such as transform tree utilities, common
/// publishers, subscribers, lifecycle related callbacks and configuration points, enabling extension by inheritance.
class BaseAMCLNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  /// Constructor, following LifecycleNode signature.
  explicit BaseAMCLNode(
      const std::string& node_name = "amcl",
      const std::string& node_namespace = "",
      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  /// Destructor for the base AMCL node.
  ~BaseAMCLNode() override;

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

  /// Get execution policy from parameters.
  auto get_execution_policy() const -> ExecutionPolicyVariant;

  /// Callback for the periodic particle updates.
  void periodic_timer_callback();

  /// Callback for the autostart timer.
  void autostart_callback();

  /// User provided extra steps for the autostart process.
  virtual void do_autostart_callback(){};

  // Configuration points for extra steps needed for the different AMCL variants.

  /// Extra steps for the on_configure callback. Defaults to no-op.
  virtual void do_configure([[maybe_unused]] const rclcpp_lifecycle::State& state) {}
  /// Extra steps for the on_deactivate callback. Defaults to no-op.
  virtual void do_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& state) {}
  /// Extra steps for the on_shutdown callback. Defaults to no-op.
  virtual void do_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& state) {}
  /// Extra steps for the on_cleanup callback. Defaults to no-op.
  virtual void do_cleanup([[maybe_unused]] const rclcpp_lifecycle::State& state) {}
  /// Extra steps for the on_activate callback. Defaults to no-op.
  virtual void do_activate([[maybe_unused]] const rclcpp_lifecycle::State& state) {}
  /// Extra steps for the periodic updates timer callback events. Defaults to no-op.
  virtual void do_periodic_timer_callback() {}
  /// Extra steps for (re)initialization messages.
  virtual void do_initial_pose_callback(
      [[maybe_unused]] geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {}

  /// Callback for (re)initialization messages.
  void initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);

  /// Particle cloud publisher.
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_cloud_pub_;
  /// Particle markers publisher.
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr particle_markers_pub_;
  /// Estimated pose publisher.
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  /// Likelihood field publisher
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr likelihood_field_pub_;
  /// Node bond with the lifecycle manager.
  std::unique_ptr<bond::Bond> bond_;
  /// Timer for periodic particle cloud updates.
  rclcpp::TimerBase::SharedPtr timer_;
  /// Transforms buffer.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  /// Transforms broadcaster.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  /// Transforms listener.
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  /// Timer for node to configure and activate itself.
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  /// Common mutually exclusive callback group.
  rclcpp::CallbackGroup::SharedPtr common_callback_group_;
  /// Common subscription options.
  rclcpp::SubscriptionOptions common_subscription_options_;
  /// Pose (re)initialization subscription.
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
};
}  // namespace beluga_amcl
#endif

// Copyright 2022 Ekumen, Inc.
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

#ifndef BELUGA_AMCL__PRIVATE__AMCL_NODE_HPP_
#define BELUGA_AMCL__PRIVATE__AMCL_NODE_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include <beluga/localization.hpp>
#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "beluga_amcl/private/execution_policy.hpp"

namespace beluga_amcl
{

class AmclNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit AmclNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~AmclNode();

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  void map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr);
  void timer_callback();
  template<typename ExecutionPolicy>
  void laser_callback(
    ExecutionPolicy && exec_policy,
    sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);
  void initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
  void reinitialize_with_pose(const Eigen::Vector3d & mean, const Eigen::Matrix3d & covariance);

  std::unique_ptr<beluga::LaserLocalizationInterface2d> particle_filter_;
  execution::Policy execution_policy_;

  std::unique_ptr<bond::Bond> bond_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr
    particle_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    likelihood_field_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
    rclcpp_lifecycle::LifecycleNode>> laser_scan_sub_;
  message_filters::Connection laser_scan_connection_;

  Sophus::SE2d last_odom_to_base_transform_;
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__PRIVATE__AMCL_NODE_HPP_

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

#ifndef BELUGA_AMCL__AMCL_NODE_HPP_
#define BELUGA_AMCL__AMCL_NODE_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include <beluga/algorithm/particle_filter.hpp>
#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/sensor/likelihood_field_model.hpp>
#include <beluga_amcl/occupancy_grid.hpp>
#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace beluga_amcl
{

class AmclNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  template<class Mixin>
  using SensorModel = typename beluga::LikelihoodFieldModel<Mixin, OccupancyGrid>;
  using ParticleFilter = beluga::AMCL<beluga::DifferentialDriveModel, SensorModel, Sophus::SE2d>;

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
  void laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr);
  /*
 * @brief Publish TF transformation from map to odom
 */
  void sendMapToOdomTransform();

  std::unique_ptr<ParticleFilter> particle_filter_;
  std::unique_ptr<bond::Bond> bond_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr
    particle_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    likelihood_field_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
    rclcpp_lifecycle::LifecycleNode>> laser_scan_sub_;
  message_filters::Connection laser_scan_connection_;

  bool latest_tf_valid_{false};
  tf2::Transform latest_tf_;


};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__AMCL_NODE_HPP_

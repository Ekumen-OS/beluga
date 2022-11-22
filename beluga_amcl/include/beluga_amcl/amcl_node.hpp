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

#include <beluga_amcl/models.hpp>

#include <beluga/beluga.h>
#include <bondcpp/bond.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>

namespace beluga_amcl
{

class AmclNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using ParticleFilter = beluga::AMCL<MockMotionModel, LikelihoodSensorModel, Pose>;

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

  std::unique_ptr<ParticleFilter> particle_filter_;
  std::unique_ptr<bond::Bond> bond_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr
    particle_cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__AMCL_NODE_HPP_

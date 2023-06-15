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

#ifndef BELUGA_AMCL__PRIVATE__AMCL_NODELET_HPP_
#define BELUGA_AMCL__PRIVATE__AMCL_NODELET_HPP_

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>

#include <message_filters/subscriber.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/SetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>

#include <memory>
#include <mutex>
#include <utility>

#include <sophus/se2.hpp>

#include "beluga_amcl/AmclConfig.h"
#include "beluga_amcl/particle_filtering.hpp"
#include "beluga_amcl/private/execution_policy.hpp"

namespace beluga_amcl {

class AmclNodelet : public nodelet::Nodelet {
 public:
  AmclNodelet() = default;
  virtual ~AmclNodelet() = default;

 protected:
  void onInit() override;

  void config_callback(beluga_amcl::AmclConfig& config, uint32_t level);

  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& message);

  void map_timer_callback(const ros::TimerEvent& ev);

  bool set_map_callback(nav_msgs::SetMap::Request& request, nav_msgs::SetMap::Response& response);

  void handle_map_with_default_initial_pose(const nav_msgs::OccupancyGrid::ConstPtr& message);

  std::unique_ptr<LaserLocalizationInterface2d> make_particle_filter(const nav_msgs::OccupancyGrid::ConstPtr& message);

  void particle_cloud_timer_callback(const ros::TimerEvent& ev);

  template <typename ExecutionPolicy>
  void laser_callback(ExecutionPolicy&& exec_policy, const sensor_msgs::LaserScan::ConstPtr& laser_scan);

  void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message);

  bool global_localization_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  void save_pose_timer_callback(const ros::TimerEvent& ev);

  void initialize_with_pose(const Sophus::SE2d& pose, const Eigen::Matrix3d& covariance);

  void update_covariance_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

  std::unique_ptr<LaserLocalizationInterface2d> particle_filter_;

  std::mutex mutex_;

  ros::Timer particle_cloud_timer_;
  ros::Publisher particle_cloud_pub_;
  ros::Publisher pose_pub_;
  ros::Timer save_pose_timer_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber map_sub_;
  ros::Timer map_timer_;
  ros::ServiceServer set_map_server_;
  ros::ServiceClient get_map_client_;
  ros::ServiceServer global_localization_server_;

  bool configured_{false};
  beluga_amcl::AmclConfig config_;
  beluga_amcl::AmclConfig default_config_;
  using AmclConfigServer = dynamic_reconfigure::Server<beluga_amcl::AmclConfig>;
  std::unique_ptr<AmclConfigServer> config_server_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  bool enable_tf_broadcast_{false};

  diagnostic_updater::Updater diagnosic_updater_;

  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_;
  message_filters::Connection laser_scan_connection_;

  nav_msgs::OccupancyGrid::ConstPtr last_known_map_;

  std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> last_known_estimate_;
  std::optional<Sophus::SE2d> latest_map_to_odom_transform_;
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__PRIVATE__AMCL_NODELET_HPP_

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

#ifndef BELUGA_AMCL_AMCL_NODELET_HPP
#define BELUGA_AMCL_AMCL_NODELET_HPP

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

#include <beluga_amcl/AmclConfig.h>
#include <beluga_ros/amcl.hpp>

/**
 * \file
 * \brief ROS (1) integration of the 2D AMCL algorithm.
 */

namespace beluga_amcl {

/// 2D AMCL as a ROS (1) nodelet.
class AmclNodelet : public nodelet::Nodelet {
 public:
  AmclNodelet() = default;
  ~AmclNodelet() override = default;

 protected:
  /// Callback for nodelet initialization.
  void onInit() override;

  /// Get initial pose estimate from parameters if set.
  auto get_initial_estimate() const -> std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>>;

  /// Get motion model as per current parametrization.
  auto get_motion_model(std::string_view) const -> beluga_ros::Amcl::motion_model_variant;

  /// Get sensor model as per current parametrization.
  auto get_sensor_model(std::string_view, const nav_msgs::OccupancyGrid::ConstPtr&) const
      -> beluga_ros::Amcl::sensor_model_variant;

  /// Get execution policy given its name.
  static auto get_execution_policy(std::string_view) -> beluga_ros::Amcl::execution_policy_variant;

  /// Instantiate particle filter given an initial occupancy grid map and the current parametrization.
  auto make_particle_filter(const nav_msgs::OccupancyGrid::ConstPtr&) const -> std::unique_ptr<beluga_ros::Amcl>;

  /// Callback for `dynamic_reconfigure` updates.
  void config_callback(beluga_amcl::AmclConfig& config, uint32_t level);

  /// Callback for occupancy grid map updates.
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& message);

  /// Callback for repeated map initialization requests.
  void map_timer_callback(const ros::TimerEvent& ev);

  /// Callback for the map update service.
  bool set_map_callback(nav_msgs::SetMap::Request& request, nav_msgs::SetMap::Response& response);

  /// Particle filter (re)initialization helper method.
  /**
   * \internal
   */
  void handle_map_with_default_initial_pose(const nav_msgs::OccupancyGrid::ConstPtr& map);

  /// Callback for periodic particle cloud updates.
  void particle_cloud_timer_callback(const ros::TimerEvent& ev);

  /// Callback for laser scan updates.
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan);

  /// Callback for pose (re)initialization.
  void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message);

  /// Callback for the global relocalization service.
  bool global_localization_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /// Callback for the no motion update service.
  bool nomotion_update_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /// Callback for periodic pose saving.
  void save_pose_timer_callback(const ros::TimerEvent& ev);

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

  /// Callback for estimated pose covariance diagnostics.
  void update_covariance_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

  /// Mutex for particle filter access.
  mutable std::mutex mutex_;

  /// Timer for periodic particle cloud updates.
  ros::Timer particle_cloud_timer_;
  /// Particle cloud publisher.
  ros::Publisher particle_cloud_pub_;
  /// Estimated pose publisher.
  ros::Publisher pose_pub_;
  /// Timer for pose saving.
  ros::Timer save_pose_timer_;
  /// Pose (re)initialization subscriber.
  ros::Subscriber initial_pose_sub_;
  /// Occupancy grid map updates subscriber
  ros::Subscriber map_sub_;

  /// Timer for repeated map initialization requests.
  ros::Timer map_timer_;
  /// Map update service server.
  ros::ServiceServer set_map_server_;
  /// Map initialization service client.
  ros::ServiceClient get_map_client_;
  /// Global relocalization service server.
  ros::ServiceServer global_localization_server_;
  /// No motion update service server.
  ros::ServiceServer nomotion_update_server_;

  /// Flag set on first configuration.
  bool configured_{false};
  /// Current `beluga_amcl` configuration.
  beluga_amcl::AmclConfig config_;
  /// Default `beluga_amcl` configuration.
  beluga_amcl::AmclConfig default_config_;
  /// Type alias for a `dynamic_reconfigure::Server` bound to `beluga_amcl` configuration.
  using AmclConfigServer = dynamic_reconfigure::Server<beluga_amcl::AmclConfig>;
  /// `beluga_amcl` configuration server.
  std::unique_ptr<AmclConfigServer> config_server_;

  /// Transforms buffer.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  /// Transforms broadcaster.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  /// Transforms listener.
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  /// Diagnostics updater.
  diagnostic_updater::Updater diagnosic_updater_;

  /// Laser scan updates subscriber.
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  /// Transform synchronization filter for laser scan updates.
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_;
  /// Connection for laser scan updates filter and callback.
  message_filters::Connection laser_scan_connection_;

  /// Particle filter instance.
  std::unique_ptr<beluga_ros::Amcl> particle_filter_;
  /// Last known pose estimate, if any.
  std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> last_known_estimate_;
  /// Last known map to odom correction estimate, if any.
  std::optional<Sophus::SE2d> last_known_odom_transform_in_map_;
  /// Last known occupancy grid map.
  nav_msgs::OccupancyGrid::ConstPtr last_known_map_;
  /// Whether to broadcast transforms or not.
  bool enable_tf_broadcast_{false};
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL_AMCL_NODELET_HPP

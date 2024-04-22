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

#ifndef BELUGA_AMCL_NDT_AMCL_NODE_HPP
#define BELUGA_AMCL_NDT_AMCL_NODE_HPP

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <beluga/sensor/ndt_sensor_model.hpp>
#include <memory>
#include <optional>
#include <utility>

#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
#include <H5Cpp.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include <beluga/beluga.hpp>
// TODO(alon): modify to use ndt_amcl.hpp
// #include <beluga_ros/amcl.hpp>

// TODO(alon): Delete this definition of beluga_ros::NdtAmcl when the original is finished.
#include <beluga_ros/laser_scan.hpp>
namespace beluga_ros {
/// Struct containing parameters for the Adaptive Monte Carlo Localization (AMCL) implementation.
struct NdtAmclParams {
  double update_min_d = 0.25;
  double update_min_a = 0.2;
  std::size_t resample_interval = 1UL;
  bool selective_resampling = false;
  std::size_t min_particles = 500UL;
  std::size_t max_particles = 2000UL;
  double alpha_slow = 0.001;
  double alpha_fast = 0.1;
  double kld_epsilon = 0.05;
  double kld_z = 3.0;
  double spatial_resolution_x = 0.5;
  double spatial_resolution_y = 0.5;
  double spatial_resolution_theta = 10 * Sophus::Constants<double>::pi() / 180;
};

/// Implementation of the Adaptive Monte Carlo Localization (AMCL) algorithm.
class NdtAmcl {
 public:
  using particle_type = std::tuple<Sophus::SE2d, beluga::Weight>;

  using motion_model_variant = std::variant<
      beluga::DifferentialDriveModel,     //
      beluga::OmnidirectionalDriveModel,  //
      beluga::StationaryModel>;

  using sparse_grid_2d_t =
      beluga::SparseValueGrid<std::unordered_map<Eigen::Vector2i, beluga::NDTCell2d, beluga::detail::CellHasher<2>>>;
  using sensor_model_variant = std::variant<beluga::NDTSensorModel<sparse_grid_2d_t>>;

  using execution_policy_variant = std::variant<std::execution::sequenced_policy, std::execution::parallel_policy>;

  /// Constructor.
  NdtAmcl(
      sparse_grid_2d_t map,
      motion_model_variant motion_model,
      sensor_model_variant sensor_model,
      const NdtAmclParams& params = NdtAmclParams{},
      execution_policy_variant execution_policy = std::execution::seq)
      : params_{params},
        map_distribution_{std::move(map)},
        motion_model_{std::move(motion_model)},
        sensor_model_{std::move(sensor_model)},
        execution_policy_{std::move(execution_policy)},
        spatial_hasher_{params_.spatial_resolution_x, params_.spatial_resolution_y, params_.spatial_resolution_theta},
        random_probability_estimator_{params_.alpha_slow, params_.alpha_fast},
        update_policy_{beluga::policies::on_motion(params_.update_min_d, params_.update_min_a)},
        resample_policy_{beluga::policies::every_n(params_.resample_interval)} {
    if (params_.selective_resampling) {
      resample_policy_ = resample_policy_ && beluga::policies::on_effective_size_drop;
    }
  }

  /// Returns a reference to the current set of particles.
  [[nodiscard]] const auto& particles() const { return particles_; }

  /// Initialize particles using a custom distribution.
  template <class Distribution>
  void initialize([[maybe_unused]] Distribution distribution) {}

  /// Initialize particles with a given pose and covariance.
  /**
   * \throw std::runtime_error If the provided covariance is invalid.
   */
  void initialize([[maybe_unused]] Sophus::SE2d pose, [[maybe_unused]] Sophus::Matrix3d covariance) {}

  /// Initialize particles using the default map distribution.
  void initialize_from_map() {}

  /// Update the map used for localization.
  void update_map([[maybe_unused]] sparse_grid_2d_t map) {}

  /// Update particles based on motion and sensor information.
  auto update([[maybe_unused]] Sophus::SE2d base_pose_in_odom, [[maybe_unused]] beluga_ros::LaserScan laser_scan)
      -> std::optional<std::pair<Sophus::SE2d, Sophus::Matrix3d>> {
    return std::pair<Sophus::SE2<double>, Sophus::Matrix3<double>>{};
  }

  /// Force a manual update of the particles on the next iteration of the filter.
  void force_update() { force_update_ = true; }

 private:
  beluga::TupleVector<particle_type> particles_;

  NdtAmclParams params_;
  sparse_grid_2d_t map_distribution_;  // TODO(alon):  Defined as sparse_grid_2d_t only for compilation propouse
  motion_model_variant motion_model_;
  sensor_model_variant sensor_model_;
  execution_policy_variant execution_policy_;

  beluga::spatial_hash<Sophus::SE2d> spatial_hasher_;
  beluga::ThrunRecoveryProbabilityEstimator random_probability_estimator_;
  beluga::any_policy<Sophus::SE2d> update_policy_;
  beluga::any_policy<decltype(particles_)> resample_policy_;

  beluga::RollingWindow<Sophus::SE2d, 2> control_action_window_;

  bool force_update_{true};
};
}  // namespace beluga_ros

namespace beluga_amcl {

class NdtAmclNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit NdtAmclNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~NdtAmclNode() override;

 protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

  auto get_initial_estimate() const -> std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>>;

  // TODO(alon): use beluga_ros::NdtAmcl::motion_model_variant
  auto get_motion_model(std::string_view) const -> beluga_ros::NdtAmcl::motion_model_variant;
  // TODO(alon): use H5Cpp map file instead of occupancyGrid
  auto get_sensor_model(std::string_view, beluga_ros::NdtAmcl::sparse_grid_2d_t) const
      -> beluga_ros::NdtAmcl::sensor_model_variant;

  static auto get_execution_policy(std::string_view) -> beluga_ros::NdtAmcl::execution_policy_variant;

  // TODO(alon): use H5Cpp map file instead of occupancyGrid
  auto make_particle_filter(beluga_ros::NdtAmcl::sparse_grid_2d_t) const -> std::unique_ptr<beluga_ros::NdtAmcl>;

  // TODO(alon): use H5Cpp map file instead of occupancyGrid
  //   void map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr);

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
   * If an exception occurs during the initialization, an error message is logged, and the initialization
   * process is also aborted, returning false. If the initialization is successful, the TF broadcast is
   * enabled.
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
  // TODO(alon): use H5Cpp map file instead of occupancyGrid
  //   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
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

  // TODO(alon): use ndt_amcl
  std::unique_ptr<beluga_ros::NdtAmcl> particle_filter_;
  std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> last_known_estimate_;
  bool enable_tf_broadcast_{false};
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL_NDT_AMCL_NODE_HPP

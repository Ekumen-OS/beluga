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

#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <beluga/beluga.hpp>
#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/motion/omnidirectional_drive_model.hpp>
#include <beluga/motion/stationary_model.hpp>
#include <beluga/primitives.hpp>
#include <beluga/sensor/data/ndt_cell.hpp>
#include <beluga/sensor/data/sparse_value_grid.hpp>
#include <beluga/sensor/ndt_sensor_model.hpp>

#include <execution>
#include <functional>
#include <optional>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/subscription_options.hpp>
#include <sophus/se2.hpp>
#include <tuple>
#include <variant>

#include <Eigen/src/Core/Matrix.h>
#include <H5Cpp.h>

#include <beluga/algorithm/amcl_core.hpp>
#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include <beluga_ros/laser_scan.hpp>
#include "beluga_amcl/message_filters_adapter.hpp"
#include "beluga_amcl/ros2_common.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include <message_filters/subscriber.hpp>
#pragma GCC diagnostic pop

/**
 * \file
 * \brief ROS 2 integration of the 2D NDT-AMCL algorithm.
 */

namespace beluga_amcl {

/// Underlying map representation for the NDT sensor model.
using NDTMapRepresentation =
    beluga::SparseValueGrid2<std::unordered_map<Eigen::Vector2i, beluga::NDTCell2d, beluga::detail::CellHasher<2>>>;

/// Type of a particle-dependent random state generator.
using RandomStateGenerator = std::function<std::function<beluga::NDTSensorModel<NDTMapRepresentation>::state_type()>(
    const beluga::TupleVector<std::tuple<beluga::NDTSensorModel<NDTMapRepresentation>::state_type, beluga::Weight>>)>;

/// Partial specialization of the core AMCL pipeline for convinience.
template <class MotionModel, class ExecutionPolicy>
using NdtAmcl = beluga::Amcl<
    MotionModel,
    beluga::NDTSensorModel<NDTMapRepresentation>,
    RandomStateGenerator,
    beluga::Weight,
    std::tuple<typename beluga::NDTSensorModel<NDTMapRepresentation>::state_type, beluga::Weight>,
    ExecutionPolicy>;

/// All combinations of supported NDT AMCL variants.
using NdtAmclVariant = std::variant<
    NdtAmcl<beluga::StationaryModel, std::execution::parallel_policy>,            //
    NdtAmcl<beluga::StationaryModel, std::execution::sequenced_policy>,           //
    NdtAmcl<beluga::DifferentialDriveModel2d, std::execution::parallel_policy>,   //
    NdtAmcl<beluga::DifferentialDriveModel2d, std::execution::sequenced_policy>,  //
    NdtAmcl<beluga::OmnidirectionalDriveModel, std::execution::parallel_policy>,  //
    NdtAmcl<beluga::OmnidirectionalDriveModel, std::execution::sequenced_policy>  //
    >;

/// Supported motion models.
using MotionModelVariant =
    std::variant<beluga::DifferentialDriveModel2d, beluga::StationaryModel, beluga::OmnidirectionalDriveModel>;

/// Supported execution policies.
using ExecutionPolicyVariant = std::variant<std::execution::sequenced_policy, std::execution::parallel_policy>;

/// 2D NDT AMCL as a ROS 2 composable lifecycle node.
class NdtAmclNode : public BaseAMCLNode {
 public:
  /// Constructor.
  explicit NdtAmclNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

 protected:
  /// Callback for lifecycle transitions from the INACTIVE state to the ACTIVE state.
  void do_activate(const rclcpp_lifecycle::State&) override;

  /// Callback for lifecycle transitions from the ACTIVE state to the INACTIVE state.
  void do_deactivate(const rclcpp_lifecycle::State&) override;

  /// Callback for lifecycle transitions from the INACTIVE state to the UNCONFIGURED state.
  void do_cleanup(const rclcpp_lifecycle::State&) override;

  /// Get initial pose estimate from parameters if set.
  auto get_initial_estimate() const -> std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>>;

  /// Get motion model as per current parametrization.
  auto get_motion_model() const -> MotionModelVariant;

  /// Get sensor model as per current parametrization.
  beluga::NDTSensorModel<NDTMapRepresentation> get_sensor_model() const;

  /// Instantiate particle filter given an initial occupancy grid map and the current parametrization.
  auto make_particle_filter() const -> std::unique_ptr<NdtAmclVariant>;

  /// Callback for periodic particle cloud updates.
  void do_periodic_timer_callback() override;

  /// Callback for laser scan updates.
  void laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr);

  /// Callback for pose (re)initialization.
  void do_initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) override;

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

  /// Laser scan updates subscription.
  using LaserScanSubscriber = compatibility::message_filters::AdaptedSubscriber<sensor_msgs::msg::LaserScan>;
  std::unique_ptr<LaserScanSubscriber> laser_scan_sub_;

  /// Transform synchronization filter for laser scan updates.
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  /// Connection for laser scan updates filter and callback.
  message_filters::Connection laser_scan_connection_;

  /// Particle filter instance.
  std::unique_ptr<NdtAmclVariant> particle_filter_ = nullptr;
  /// Last known pose estimate, if any.
  std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> last_known_estimate_ = std::nullopt;
  /// Last known map to odom correction estimate, if any.
  std::optional<Sophus::SE2d> last_known_odom_transform_in_map_;
  /// Whether to broadcast transforms or not.
  bool enable_tf_broadcast_{false};
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL_NDT_AMCL_NODE_HPP

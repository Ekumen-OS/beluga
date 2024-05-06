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
#include <sophus/se2.hpp>
#include <tuple>
#include <variant>

#include <H5Cpp.h>
#include <beluga/algorithm/amcl_core.hpp>
#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <message_filters/subscriber.h>
#include <beluga_ros/laser_scan.hpp>

namespace beluga_amcl {

// Underlying map representation for the NDT sensor model.
using NDTMapRepresentation =
    beluga::SparseValueGrid<std::unordered_map<Eigen::Vector2i, beluga::NDTCell2d, beluga::detail::CellHasher<2>>>;

// Partial specialization of the core AMCL pipeline for convinience.
template <class MotionModel, class ExecutionPolicy>
using NdtAmcl = beluga::Amcl<
    MotionModel,
    beluga::NDTSensorModel<NDTMapRepresentation>,
    std::function<beluga::NDTSensorModel<NDTMapRepresentation>::state_type()>,
    beluga::Weight,
    std::tuple<typename beluga::NDTSensorModel<NDTMapRepresentation>::state_type, beluga::Weight>,
    ExecutionPolicy>;

// All combinations of supported NDT AMCL variants.
using NdtAmclVariant = std::variant<
    NdtAmcl<beluga::StationaryModel, std::execution::parallel_policy>,            //
    NdtAmcl<beluga::StationaryModel, std::execution::sequenced_policy>,           //
    NdtAmcl<beluga::DifferentialDriveModel, std::execution::parallel_policy>,     //
    NdtAmcl<beluga::DifferentialDriveModel, std::execution::sequenced_policy>,    //
    NdtAmcl<beluga::OmnidirectionalDriveModel, std::execution::parallel_policy>,  //
    NdtAmcl<beluga::OmnidirectionalDriveModel, std::execution::sequenced_policy>  //
    >;

// Supported motion models.
using MotionModelVariant =
    std::variant<beluga::DifferentialDriveModel, beluga::StationaryModel, beluga::OmnidirectionalDriveModel>;

// Supported execution policies.
using ExecutionPolicyVariant = std::variant<std::execution::sequenced_policy, std::execution::parallel_policy>;

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

  auto get_motion_model() const -> MotionModelVariant;

  auto get_execution_policy() const -> ExecutionPolicyVariant;

  beluga::NDTSensorModel<NDTMapRepresentation> get_sensor_model() const;

  auto make_particle_filter() const -> std::unique_ptr<NdtAmclVariant>;

  void timer_callback();

  void laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr);

  void initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);

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

  std::unique_ptr<bond::Bond> bond_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>>
      laser_scan_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  message_filters::Connection laser_scan_connection_;

  std::unique_ptr<NdtAmclVariant> particle_filter_ = nullptr;
  std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> last_known_estimate_ = std::nullopt;
  bool enable_tf_broadcast_{false};
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL_NDT_AMCL_NODE_HPP

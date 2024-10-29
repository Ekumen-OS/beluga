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

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <beluga_ros/particle_cloud.hpp>
#include <beluga_ros/tf2_sophus.hpp>

#include <beluga_amcl/ndt_amcl_node.hpp>

namespace beluga_amcl {

NdtAmclNode::NdtAmclNode(const rclcpp::NodeOptions& options) : BaseAMCLNode{"ndt_amcl", "", options} {
  RCLCPP_INFO(get_logger(), "Creating");
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Likelihood for measurements that lie inside cells that are not present in the map.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("minimum_likelihood", rclcpp::ParameterValue(0.01), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Scaling parameter d1 in literature, used for scaling 2D likelihoods.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1000;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("d1", rclcpp::ParameterValue(1.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Scaling parameter d2 in literature, used for scaling 2D likelihoods.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1000;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("d2", rclcpp::ParameterValue(0.6), descriptor);
  }
}

void NdtAmclNode::do_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Making particle filter");
  particle_filter_ = make_particle_filter();

  {
    using LaserScanSubscriber =
        message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>;
    laser_scan_sub_ = std::make_unique<LaserScanSubscriber>(
        shared_from_this(), get_parameter("scan_topic").as_string(), rmw_qos_profile_sensor_data,
        common_subscription_options_);

    // Message filter that caches laser scan readings until it is possible to transform
    // from laser frame to odom frame and update the particle filter.
    laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *laser_scan_sub_, *tf_buffer_, get_parameter("odom_frame_id").as_string(), 10, get_node_logging_interface(),
        get_node_clock_interface(), tf2::durationFromSec(get_parameter("transform_tolerance").as_double()));

    laser_scan_connection_ =
        laser_scan_filter_->registerCallback(std::bind(&NdtAmclNode::laser_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to scan_topic: %s", laser_scan_sub_->getTopic().c_str());

    const auto initial_estimate = get_initial_estimate();
    if (initial_estimate.has_value()) {
      last_known_estimate_ = initial_estimate;
      last_known_odom_transform_in_map_.reset();
      initialize_from_estimate(initial_estimate.value());
    }
  }
}

void NdtAmclNode::do_deactivate(const rclcpp_lifecycle::State&) {
  laser_scan_connection_.disconnect();
  laser_scan_filter_.reset();
  laser_scan_sub_.reset();
}

void NdtAmclNode::do_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Cleaning up");
  particle_cloud_pub_.reset();
  pose_pub_.reset();
  particle_filter_.reset();
  enable_tf_broadcast_ = false;
}

auto NdtAmclNode::get_initial_estimate() const -> std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> {
  if (!get_parameter("set_initial_pose").as_bool()) {
    return std::nullopt;
  }

  const auto pose = Sophus::SE2d{
      Sophus::SO2d{get_parameter("initial_pose.yaw").as_double()},
      Eigen::Vector2d{
          get_parameter("initial_pose.x").as_double(),
          get_parameter("initial_pose.y").as_double(),
      },
  };

  Eigen::Matrix3d covariance;
  covariance.coeffRef(0, 0) = get_parameter("initial_pose.covariance_x").as_double();
  covariance.coeffRef(1, 1) = get_parameter("initial_pose.covariance_y").as_double();
  covariance.coeffRef(2, 2) = get_parameter("initial_pose.covariance_yaw").as_double();
  covariance.coeffRef(0, 1) = get_parameter("initial_pose.covariance_xy").as_double();
  covariance.coeffRef(1, 0) = covariance.coeffRef(0, 1);
  covariance.coeffRef(0, 2) = get_parameter("initial_pose.covariance_xyaw").as_double();
  covariance.coeffRef(2, 0) = covariance.coeffRef(0, 2);
  covariance.coeffRef(1, 2) = get_parameter("initial_pose.covariance_yyaw").as_double();
  covariance.coeffRef(2, 1) = covariance.coeffRef(1, 2);

  return std::make_pair(pose, covariance);
}

auto NdtAmclNode::get_motion_model() const -> beluga::DifferentialDriveModel2d {
  const auto name = get_parameter("robot_model_type").as_string();
  if (name == kDifferentialModelName) {
    auto params = beluga::DifferentialDriveModelParam{};
    params.rotation_noise_from_rotation = get_parameter("alpha1").as_double();
    params.rotation_noise_from_translation = get_parameter("alpha2").as_double();
    params.translation_noise_from_translation = get_parameter("alpha3").as_double();
    params.translation_noise_from_rotation = get_parameter("alpha4").as_double();
    return beluga::DifferentialDriveModel2d{params};
  }
  throw std::invalid_argument(std::string("Invalid motion model: ") + name);
}

auto NdtAmclNode::get_sensor_model() const -> beluga::NdtSensorModel<NdtMap> {
  auto params = beluga::NdtModelParam2d{};
  params.minimum_likelihood = get_parameter("minimum_likelihood").as_double();
  params.d1 = get_parameter("d1").as_double();
  params.d2 = get_parameter("d2").as_double();
  const auto map_path = get_parameter("map_path").as_string();
  RCLCPP_INFO(get_logger(), "Loading map from %s.", map_path.c_str());

  return beluga::NdtSensorModel<NdtMap>{
      params, beluga::io::load_from_hdf5<NdtMap>(get_parameter("map_path").as_string())};
}

auto NdtAmclNode::make_particle_filter() const -> std::unique_ptr<beluga_amcl::NdtAmcl> {
  auto params = beluga::AmclParams{};
  params.update_min_d = get_parameter("update_min_d").as_double();
  params.update_min_a = get_parameter("update_min_a").as_double();
  params.resample_interval = static_cast<std::size_t>(get_parameter("resample_interval").as_int());
  params.selective_resampling = get_parameter("selective_resampling").as_bool();
  params.min_particles = static_cast<std::size_t>(get_parameter("min_particles").as_int());
  params.max_particles = static_cast<std::size_t>(get_parameter("max_particles").as_int());
  params.kld_epsilon = get_parameter("pf_err").as_double();
  params.kld_z = get_parameter("pf_z").as_double();

  auto hasher = beluga::spatial_hash<Sophus::SE2d>(
      get_parameter("spatial_resolution_x").as_double(),  //
      get_parameter("spatial_resolution_y").as_double(),  //
      get_parameter("spatial_resolution_theta").as_double());

  return std::make_unique<beluga_amcl::NdtAmcl>(
      get_motion_model(),
      get_sensor_model(),  //
      std::move(hasher),   //
      params);
}

void NdtAmclNode::do_periodic_timer_callback() {
  if (!particle_filter_) {
    return;
  }

  if (particle_cloud_pub_->get_subscription_count() == 0) {
    return;
  }

  auto message = beluga_ros::msg::PoseArray{};
  beluga_ros::assign_particle_cloud(particle_filter_->particles(), message);
  beluga_ros::stamp_message(get_parameter("global_frame_id").as_string(), now(), message);
  particle_cloud_pub_->publish(message);
}

// TODO(alon): Wouldn't it be better in the callback of each message to simply receive
// it and define another timer or thread to do the work of calculation and publication?
void NdtAmclNode::laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
  if (!particle_filter_ || !last_known_estimate_.has_value()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring laser data because the particle filter has not been initialized");
    return;
  }

  auto base_pose_in_odom = Sophus::SE2d{};
  try {
    // Use the lookupTransform overload with no timeout since we're not using a dedicated
    // tf thread. The message filter we are using avoids the need for it.
    tf2::convert(
        tf_buffer_
            ->lookupTransform(
                get_parameter("odom_frame_id").as_string(), get_parameter("base_frame_id").as_string(),
                tf2_ros::fromMsg(laser_scan->header.stamp))
            .transform,
        base_pose_in_odom);
  } catch (const tf2::TransformException& error) {
    RCLCPP_ERROR(get_logger(), "Could not transform from odom to base: %s", error.what());
    return;
  }

  auto laser_pose_in_base = Sophus::SE3d{};
  try {
    tf2::convert(
        tf_buffer_
            ->lookupTransform(
                get_parameter("base_frame_id").as_string(), laser_scan->header.frame_id,
                tf2_ros::fromMsg(laser_scan->header.stamp))
            .transform,
        laser_pose_in_base);
  } catch (const tf2::TransformException& error) {
    RCLCPP_ERROR(get_logger(), "Could not transform from base to laser: %s", error.what());
    return;
  }

  const auto update_start_time = std::chrono::high_resolution_clock::now();

  // TODO(nahuel): Remove this once we update the measurement type.
  auto scan = beluga_ros::LaserScan{
      laser_scan,                                                     //
      laser_pose_in_base,                                             //
      static_cast<std::size_t>(get_parameter("max_beams").as_int()),  //
      get_parameter("laser_min_range").as_double(),                   //
      get_parameter("laser_max_range").as_double()};

  auto measurement = scan.points_in_cartesian_coordinates() |  //
                     ranges::views::transform([&scan](const auto& p) {
                       return Eigen::Vector2d((scan.origin() * Sophus::Vector3d{p.x(), p.y(), 0}).head<2>());
                     }) |
                     ranges::to<std::vector>;

  const auto new_estimate = particle_filter_->update(base_pose_in_odom, std::move(measurement));

  const auto update_stop_time = std::chrono::high_resolution_clock::now();
  const auto update_duration = update_stop_time - update_start_time;

  if (new_estimate.has_value()) {
    const auto& [base_pose_in_map, _] = new_estimate.value();
    last_known_odom_transform_in_map_ = base_pose_in_map * base_pose_in_odom.inverse();
    last_known_estimate_ = new_estimate;

    const auto num_particles = particle_filter_->particles().size();

    RCLCPP_INFO(
        get_logger(), "Particle filter update iteration stats: %ld particles %ld points - %.3fms", num_particles,
        laser_scan->ranges.size(), std::chrono::duration<double, std::milli>(update_duration).count());
  }

  if (!last_known_estimate_.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Estimate not available for publishing");
    return;
  }

  // Transforms are always published to keep them current.
  if (enable_tf_broadcast_ && get_parameter("tf_broadcast").as_bool()) {
    if (last_known_odom_transform_in_map_.has_value()) {
      auto message = geometry_msgs::msg::TransformStamped{};
      // Sending a transform that is valid into the future so that odom can be used.
      const auto expiration_stamp = tf2_ros::fromMsg(laser_scan->header.stamp) +
                                    tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
      message.header.stamp = tf2_ros::toMsg(expiration_stamp);
      message.header.frame_id = get_parameter("global_frame_id").as_string();
      message.child_frame_id = get_parameter("odom_frame_id").as_string();
      message.transform = tf2::toMsg(last_known_odom_transform_in_map_.value());
      tf_broadcaster_->sendTransform(message);
    }
  }

  // New pose messages are only published on updates to the filter.
  if (new_estimate.has_value()) {
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped{};
    message.header.stamp = laser_scan->header.stamp;
    message.header.frame_id = get_parameter("global_frame_id").as_string();
    const auto& [base_pose_in_map, base_pose_covariance] = new_estimate.value();
    tf2::toMsg(base_pose_in_map, message.pose.pose);
    tf2::covarianceEigenToRowMajor(base_pose_covariance, message.pose.covariance);
    pose_pub_->publish(message);
  }
}

void NdtAmclNode::do_initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
  auto pose = Sophus::SE2d{};
  tf2::convert(message->pose.pose, pose);

  auto covariance = Eigen::Matrix3d{};
  tf2::covarianceRowMajorToEigen(message->pose.covariance, covariance);

  last_known_estimate_ = std::make_pair(pose, covariance);
  last_known_odom_transform_in_map_.reset();
  initialize_from_estimate(last_known_estimate_.value());
}

bool NdtAmclNode::initialize_from_estimate(const std::pair<Sophus::SE2d, Eigen::Matrix3d>& estimate) {
  RCLCPP_INFO(get_logger(), "Initializing particles from estimated pose and covariance");

  if (particle_filter_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particles: The particle filter has not been initialized");
    return false;
  }

  try {
    particle_filter_->initialize(estimate.first, estimate.second);
  } catch (const std::runtime_error& error) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particles: %s", error.what());
    return false;
  }

  enable_tf_broadcast_ = true;

  const auto& pose = estimate.first;
  RCLCPP_INFO(
      get_logger(), "Particle filter initialized with %ld particles about initial pose x=%g, y=%g, yaw=%g",
      particle_filter_->particles().size(),  //
      pose.translation().x(),                //
      pose.translation().y(),                //
      pose.so2().log());

  return true;
}

}  // namespace beluga_amcl

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(beluga_amcl::NdtAmclNode)

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

#include <chrono>
#include <cstddef>
#include <execution>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <ratio>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/buffer_interface.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <Eigen/Core>
#include <sophus/se2.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include <message_filters/subscriber.hpp>
#pragma GCC diagnostic pop

#include <rclcpp/version.h>
#include <bondcpp/bond.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <range/v3/algorithm/all_of.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_srvs/srv/empty.hpp>

#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/motion/omnidirectional_drive_model.hpp>
#include <beluga/motion/stationary_model.hpp>
#include <beluga/sensor/beam_model.hpp>
#include <beluga/sensor/likelihood_field_model.hpp>
#include <beluga/sensor/likelihood_field_prob_model.hpp>
#include <beluga_ros/amcl.hpp>
#include <beluga_ros/likelihood_field.hpp>
#include <beluga_ros/messages.hpp>
#include <beluga_ros/particle_cloud.hpp>
#include <beluga_ros/tf2_sophus.hpp>
#include "beluga_amcl/amcl_node.hpp"
#include "beluga_amcl/message_filters.hpp"
#include "beluga_amcl/ros2_common.hpp"

namespace beluga_amcl {

namespace {

constexpr std::string_view kLikelihoodFieldModelName = "likelihood_field";
constexpr std::string_view kLikelihoodFieldProbModelName = "likelihood_field_prob";
constexpr std::string_view kBeamSensorModelName = "beam";

}  // namespace

AmclNode::AmclNode(const rclcpp::NodeOptions& options) : BaseAMCLNode{"amcl", "", options} {
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Topic to subscribe to in order to receive the map to localize on.";
    declare_parameter("map_topic", rclcpp::ParameterValue("map"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Which observation model to use [beam, likelihood_field].";
    declare_parameter("laser_model_type", rclcpp::ParameterValue(std::string(kLikelihoodFieldModelName)), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Maximum distance to do obstacle inflation on map, used in likelihood field model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("laser_likelihood_max_dist", rclcpp::ParameterValue(2.0), descriptor);
  }
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Mixture weight for the probability of hitting an obstacle.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("z_hit", rclcpp::ParameterValue(0.5), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Mixture weight for the probability of getting random measurements.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("z_rand", rclcpp::ParameterValue(0.5), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Whether to model unknown space or assume it free.";
    declare_parameter("model_unknown_space", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Whether to pre-process thick walls, keeping borders and setting inner cells to unknown space, or not.";
    declare_parameter("pre_process_thick_walls", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Mixture weight for the probability of getting max range measurements.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("z_max", rclcpp::ParameterValue(0.05), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Mixture weight for the probability of getting short measurements.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("z_short", rclcpp::ParameterValue(0.05), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Short readings' exponential distribution parameter.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("lambda_short", rclcpp::ParameterValue(0.1), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Standard deviation of the hit distribution.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("sigma_hit", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "If false, AMCL will use the last known pose to initialize when a new map is received.";
    declare_parameter("always_reset_initial_pose", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Set this to true when you want to load only the first published map from map_server "
        "and ignore subsequent ones.";
    declare_parameter("first_map_only", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Set this to true to enable debugging aids. This will "
        "increase resource usage and potentially degrade performance.";
    declare_parameter("debug", false, descriptor);
  }
}

AmclNode::~AmclNode() {
  RCLCPP_INFO(get_logger(), "Destroying");
  // In case this lifecycle node wasn't properly shut down, do it here.
  on_shutdown(get_current_state());
}

void AmclNode::do_activate(const rclcpp_lifecycle::State&) {
  // Ensure likelihood field publisher is (re)activated early enough.
  if (likelihood_field_pub_) {
    likelihood_field_pub_->on_activate();
  }

  {
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        get_parameter("map_topic").as_string(), rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&AmclNode::map_callback, this, std::placeholders::_1), common_subscription_options_);
    RCLCPP_INFO(get_logger(), "Subscribed to map_topic: %s", map_sub_->get_topic_name());
  }

  const auto sensor_qos = [] {
    if constexpr (BELUGA_AMCL_MESSAGE_FILTERS_VERSION_GTE(7, 2, 1)) {
      return rclcpp::SensorDataQoS();
    } else {
      return rmw_qos_profile_sensor_data;
    }
  }();

  const auto scan_topic = get_parameter("scan_topic").as_string();
  const auto point_cloud_topic = get_parameter("point_cloud_topic").as_string();

  if ((!scan_topic.empty() && !point_cloud_topic.empty())) {
    RCLCPP_ERROR(get_logger(), "scan_topic and point_cloud_topic cannot be specified at the same time");
    throw std::invalid_argument("scan_topic and point_cloud_topic cannot be specified at the same time");
  }

  if (!point_cloud_topic.empty()) {
    point_cloud_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        shared_from_this(), point_cloud_topic, sensor_qos, common_subscription_options_);

    point_cloud_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *point_cloud_sub_, *tf_buffer_, get_parameter("odom_frame_id").as_string(), 10, get_node_logging_interface(),
        get_node_clock_interface(), tf2::durationFromSec(get_parameter("transform_tolerance").as_double()));

    point_cloud_connection_ = point_cloud_filter_->registerCallback(
        std::bind(&AmclNode::sensor_callback<sensor_msgs::msg::PointCloud2>, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to point_cloud_topic: %s", point_cloud_sub_->getTopic().c_str());
  } else {
    const auto effective_scan_topic = scan_topic.empty() ? "scan" : scan_topic;
    if (scan_topic.empty()) {
      RCLCPP_INFO(get_logger(), "No scan_topic specified, defaulting to: %s", effective_scan_topic.c_str());
    }
    laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
        shared_from_this(), effective_scan_topic, sensor_qos, common_subscription_options_);

    laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *laser_scan_sub_, *tf_buffer_, get_parameter("odom_frame_id").as_string(), 10, get_node_logging_interface(),
        get_node_clock_interface(), tf2::durationFromSec(get_parameter("transform_tolerance").as_double()));

    laser_scan_connection_ = laser_scan_filter_->registerCallback(
        std::bind(&AmclNode::sensor_callback<sensor_msgs::msg::LaserScan>, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to scan_topic: %s", laser_scan_sub_->getTopic().c_str());
  }

  const auto common_service_qos = [] {
    if constexpr (RCLCPP_VERSION_GTE(17, 0, 0)) {
      return rclcpp::ServicesQoS();
    } else {
      return rmw_qos_profile_services_default;
    }
  }();

  global_localization_server_ = create_service<std_srvs::srv::Empty>(
      "reinitialize_global_localization",
      std::bind(
          &AmclNode::global_localization_callback, this, std::placeholders::_1, std::placeholders::_2,
          std::placeholders::_3),
      common_service_qos, common_callback_group_);
  RCLCPP_INFO(get_logger(), "Created reinitialize_global_localization service");

  nomotion_update_server_ = create_service<std_srvs::srv::Empty>(
      "request_nomotion_update",
      std::bind(
          &AmclNode::nomotion_update_callback, this, std::placeholders::_1, std::placeholders::_2,
          std::placeholders::_3),
      common_service_qos, common_callback_group_);
  RCLCPP_INFO(get_logger(), "Created request_nomotion_update service");
}

void AmclNode::do_deactivate(const rclcpp_lifecycle::State&) {
  // Reset subscriptions.
  map_sub_.reset();
  // Reset services.
  global_localization_server_.reset();
  nomotion_update_server_.reset();
  // Disconnect the callbacks for sensor data to stop processing them.
  if (laser_scan_sub_) {
    laser_scan_connection_.disconnect();
    laser_scan_filter_.reset();
    laser_scan_sub_.reset();
  }
  if (point_cloud_sub_) {
    point_cloud_connection_.disconnect();
    point_cloud_filter_.reset();
    point_cloud_sub_.reset();
  }
  if (likelihood_field_pub_) {
    likelihood_field_pub_->on_deactivate();
  }
}

void AmclNode::do_cleanup(const rclcpp_lifecycle::State&) {
  // Release all resources.
  particle_filter_.reset();
  enable_tf_broadcast_ = false;
  likelihood_field_pub_.reset();
}

auto AmclNode::get_initial_estimate() const -> std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> {
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

auto AmclNode::get_motion_model(std::string_view name) const -> beluga_ros::Amcl::motion_model_variant {
  if (name == kDifferentialModelName || name == kNav2DifferentialModelName) {
    auto params = beluga::DifferentialDriveModelParam{};
    params.rotation_noise_from_rotation = get_parameter("alpha1").as_double();
    params.rotation_noise_from_translation = get_parameter("alpha2").as_double();
    params.translation_noise_from_translation = get_parameter("alpha3").as_double();
    params.translation_noise_from_rotation = get_parameter("alpha4").as_double();
    return beluga::DifferentialDriveModel{params};
  }
  if (name == kOmnidirectionalModelName || name == kNav2OmnidirectionalModelName) {
    auto params = beluga::OmnidirectionalDriveModelParam{};
    params.rotation_noise_from_rotation = get_parameter("alpha1").as_double();
    params.rotation_noise_from_translation = get_parameter("alpha2").as_double();
    params.translation_noise_from_translation = get_parameter("alpha3").as_double();
    params.translation_noise_from_rotation = get_parameter("alpha4").as_double();
    params.strafe_noise_from_translation = get_parameter("alpha5").as_double();
    return beluga::OmnidirectionalDriveModel{params};
  }
  if (name == kStationaryModelName) {
    return beluga::StationaryModel{};
  }
  throw std::invalid_argument(std::string("Invalid motion model: ") + std::string(name));
}

auto AmclNode::get_sensor_model(std::string_view name, nav_msgs::msg::OccupancyGrid::SharedPtr map) const
    -> beluga_ros::Amcl::sensor_model_variant {
  if (name == kLikelihoodFieldModelName) {
    auto params = beluga::LikelihoodFieldModelParam{};
    params.max_obstacle_distance = get_parameter("laser_likelihood_max_dist").as_double();
    params.max_laser_distance = get_parameter("laser_max_range").as_double();
    params.z_hit = get_parameter("z_hit").as_double();
    params.z_random = get_parameter("z_rand").as_double();
    params.sigma_hit = get_parameter("sigma_hit").as_double();
    params.model_unknown_space = get_parameter("model_unknown_space").as_bool();
    params.pre_process_thick_walls = get_parameter("pre_process_thick_walls").as_bool();
    return beluga::LikelihoodFieldModel{params, beluga_ros::OccupancyGrid{map}};
  }
  if (name == kLikelihoodFieldProbModelName) {
    auto params = beluga::LikelihoodFieldProbModelParam{};
    params.max_obstacle_distance = get_parameter("laser_likelihood_max_dist").as_double();
    params.max_laser_distance = get_parameter("laser_max_range").as_double();
    params.z_hit = get_parameter("z_hit").as_double();
    params.z_random = get_parameter("z_rand").as_double();
    params.sigma_hit = get_parameter("sigma_hit").as_double();
    return beluga::LikelihoodFieldProbModel{params, beluga_ros::OccupancyGrid{map}};
  }
  if (name == kBeamSensorModelName) {
    auto params = beluga::BeamModelParam{};
    params.z_hit = get_parameter("z_hit").as_double();
    params.z_short = get_parameter("z_short").as_double();
    params.z_max = get_parameter("z_max").as_double();
    params.z_rand = get_parameter("z_rand").as_double();
    params.sigma_hit = get_parameter("sigma_hit").as_double();
    params.lambda_short = get_parameter("lambda_short").as_double();
    params.beam_max_range = get_parameter("laser_max_range").as_double();
    return beluga::BeamSensorModel{params, beluga_ros::OccupancyGrid{map}};
  }
  throw std::invalid_argument(std::string("Invalid sensor model: ") + std::string(name));
}

auto AmclNode::make_particle_filter(nav_msgs::msg::OccupancyGrid::SharedPtr map) const
    -> std::unique_ptr<beluga_ros::Amcl> {
  auto params = beluga_ros::AmclParams{};
  params.update_min_d = get_parameter("update_min_d").as_double();
  params.update_min_a = get_parameter("update_min_a").as_double();
  params.resample_interval = static_cast<std::size_t>(get_parameter("resample_interval").as_int());
  params.selective_resampling = get_parameter("selective_resampling").as_bool();
  params.min_particles = static_cast<std::size_t>(get_parameter("min_particles").as_int());
  params.max_particles = static_cast<std::size_t>(get_parameter("max_particles").as_int());
  params.alpha_slow = get_parameter("recovery_alpha_slow").as_double();
  params.alpha_fast = get_parameter("recovery_alpha_fast").as_double();
  params.kld_epsilon = get_parameter("pf_err").as_double();
  params.kld_z = get_parameter("pf_z").as_double();
  params.spatial_resolution_x = get_parameter("spatial_resolution_x").as_double();
  params.spatial_resolution_y = get_parameter("spatial_resolution_y").as_double();
  params.spatial_resolution_theta = get_parameter("spatial_resolution_theta").as_double();

  return std::make_unique<beluga_ros::Amcl>(
      beluga_ros::OccupancyGrid{map},                                        //
      get_motion_model(get_parameter("robot_model_type").as_string()),       //
      get_sensor_model(get_parameter("laser_model_type").as_string(), map),  //
      params,                                                                //
      get_execution_policy());
}

void AmclNode::map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  RCLCPP_INFO(get_logger(), "A new map was received");

  if (particle_filter_ && get_parameter("first_map_only").as_bool()) {
    RCLCPP_WARN(get_logger(), "Ignoring new map because the particle filter has already been initialized");
    return;
  }

  const auto global_frame_id = get_parameter("global_frame_id").as_string();
  if (map->header.frame_id != global_frame_id) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Map frame \"%s\" doesn't match global frame \"%s\"",
        map->header.frame_id.c_str(), global_frame_id.c_str());
  }

  const bool should_reset_initial_pose = get_parameter("always_reset_initial_pose").as_bool() ||  //
                                         (!particle_filter_ && !last_known_estimate_.has_value());

  if (!particle_filter_) {
    try {
      RCLCPP_INFO(get_logger(), "Initializing particle filter instance");
      particle_filter_ = make_particle_filter(std::move(map));
      RCLCPP_INFO(get_logger(), "Particle filter initialization completed");
    } catch (const std::invalid_argument& error) {
      RCLCPP_ERROR(get_logger(), "Could not initialize particle filter: %s", error.what());
      return;
    }
    if (get_parameter("debug").as_bool() && particle_filter_->has_likelihood_field()) {
      auto qos = rclcpp::SystemDefaultsQoS();
      qos.reliable().transient_local();
      likelihood_field_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("likelihood_field", qos);
      // Activate publisher immediately, we are likely past the activation phase.
      likelihood_field_pub_->on_activate();
    }
  } else {
    particle_filter_->update_map(beluga_ros::OccupancyGrid{std::move(map)});
  }

  if (likelihood_field_pub_) {
    auto message = nav_msgs::msg::OccupancyGrid{};
    beluga_ros::assign_likelihood_field(
        particle_filter_->likelihood_field(), particle_filter_->likelihood_field_origin(), message);
    beluga_ros::stamp_message(get_parameter("global_frame_id").as_string(), now(), message);
    likelihood_field_pub_->publish(message);
  }

  if (should_reset_initial_pose) {
    const auto initial_estimate = get_initial_estimate();
    if (initial_estimate.has_value()) {
      last_known_estimate_ = initial_estimate;
      last_known_odom_transform_in_map_.reset();
    }
  }

  if (last_known_estimate_.has_value() && initialize_from_estimate(last_known_estimate_.value())) {
    return;  // Success!
  }

  initialize_from_map();

  // TF broadcasting should be enabled only if we initialize from an estimate or in response
  // to external global localization requests, and not during the initial setup of the filter.
  enable_tf_broadcast_ = false;
}

void AmclNode::do_periodic_timer_callback() {
  if (!particle_filter_) {
    return;
  }

  if (particle_cloud_pub_->get_subscription_count() > 0) {
    auto message = geometry_msgs::msg::PoseArray{};
    beluga_ros::assign_particle_cloud(particle_filter_->particles(), message);
    beluga_ros::stamp_message(get_parameter("global_frame_id").as_string(), now(), message);
    particle_cloud_pub_->publish(message);
  }

  if (particle_markers_pub_->get_subscription_count() > 0) {
    auto message = visualization_msgs::msg::MarkerArray{};
    beluga_ros::assign_particle_cloud(particle_filter_->particles(), message);
    beluga_ros::stamp_message(get_parameter("global_frame_id").as_string(), now(), message);
    particle_markers_pub_->publish(message);
  }
}

template <typename TransformT>
std::optional<TransformT> AmclNode::lookup_transform(
    const std::string& target_frame_id,
    const std::string& source_frame_id,
    const tf2::TimePoint& stamp) {
  try {
    auto output = TransformT{};
    tf2::convert(tf_buffer_->lookupTransform(target_frame_id, source_frame_id, stamp).transform, output);
    return output;
  } catch (const tf2::TransformException& error) {
    RCLCPP_ERROR(
        get_logger(), "Could not transform from %s to %s: %s", target_frame_id.c_str(), source_frame_id.c_str(),
        error.what());
    return std::nullopt;
  }
}

std::optional<beluga_ros::LaserScan> AmclNode::wrap_sensor_data(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& sensor_msg) {
  auto sensor_pose_in_base = lookup_transform<Sophus::SE3d>(
      get_parameter("base_frame_id").as_string(), sensor_msg->header.frame_id,
      tf2_ros::fromMsg(sensor_msg->header.stamp));
  if (!sensor_pose_in_base.has_value()) {
    RCLCPP_ERROR(get_logger(), "Could not transform from base to sensor");
    return std::nullopt;
  }

  const auto max_beams = static_cast<std::size_t>(get_parameter("max_beams").as_int());
  const double min_range = get_parameter("laser_min_range").as_double();
  const double max_range = get_parameter("laser_max_range").as_double();
  return beluga_ros::LaserScan{sensor_msg, sensor_pose_in_base.value(), max_beams, min_range, max_range};
}

std::optional<beluga_ros::SparsePointCloud3f> AmclNode::wrap_sensor_data(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& sensor_msg) {
  auto sensor_pose_in_base = lookup_transform<Sophus::SE3d>(
      get_parameter("base_frame_id").as_string(), sensor_msg->header.frame_id,
      tf2_ros::fromMsg(sensor_msg->header.stamp));
  if (!sensor_pose_in_base.has_value()) {
    RCLCPP_ERROR(get_logger(), "Could not transform from base to sensor");
    return std::nullopt;
  }

  auto pointcloud = beluga_ros::SparsePointCloud3f{sensor_msg, sensor_pose_in_base.value()};

  if (pointcloud.size() > 0) {
    static std::once_flag flag;
    std::call_once(flag, [&] {
      const Eigen::Vector3f head = *pointcloud.points().begin();
      const auto on_constant_z_plane = [=](const auto& point) {
        return std::fabs(point.z() - head.z()) < std::numeric_limits<float>::epsilon();
      };
      if (!ranges::all_of(pointcloud.points(), on_constant_z_plane)) {
        RCLCPP_WARN(get_logger(), "Point cloud is NOT on a z = constant plane, filter will misbehave");
      }
    });
  }
  return pointcloud;
}

template <typename MessageT>
void AmclNode::sensor_callback(const std::shared_ptr<const MessageT>& sensor_msg) {
  if (!particle_filter_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring sensor data because the particle filter has not been initialized");
    return;
  }

  auto base_pose_in_odom = lookup_transform<Sophus::SE2d>(
      get_parameter("odom_frame_id").as_string(), get_parameter("base_frame_id").as_string(),
      tf2_ros::fromMsg(sensor_msg->header.stamp));
  if (!base_pose_in_odom.has_value()) {
    RCLCPP_ERROR(get_logger(), "Failed to lookup motion data");
    return;
  }

  auto measurement = wrap_sensor_data(sensor_msg);
  if (!measurement.has_value()) {
    RCLCPP_ERROR(get_logger(), "Failed to process sensor data");
    return;
  }

  const auto update_start_time = std::chrono::high_resolution_clock::now();
  const auto new_estimate = particle_filter_->update(base_pose_in_odom.value(), measurement.value());
  const auto update_stop_time = std::chrono::high_resolution_clock::now();
  const auto update_duration = update_stop_time - update_start_time;

  if (new_estimate.has_value()) {
    const auto& [base_pose_in_map, _] = new_estimate.value();
    last_known_odom_transform_in_map_ = base_pose_in_map * base_pose_in_odom->inverse();
    last_known_estimate_ = new_estimate;

    RCLCPP_INFO(
        get_logger(), "Particle filter update iteration stats: %ld particles %ld points - %.3fms",
        particle_filter_->particles().size(), measurement->size(),
        std::chrono::duration<double, std::milli>(update_duration).count());
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
      const auto expiration_stamp = tf2_ros::fromMsg(sensor_msg->header.stamp) +
                                    tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
      message.header.stamp = tf2_ros::toMsg(expiration_stamp);
      message.header.frame_id = get_parameter("global_frame_id").as_string();
      message.child_frame_id = get_parameter("odom_frame_id").as_string();
      message.transform = tf2::toMsg(*last_known_odom_transform_in_map_);
      tf_broadcaster_->sendTransform(message);
    }
  }

  // New pose messages are only published on updates to the filter.
  if (new_estimate.has_value()) {
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped{};
    message.header.stamp = sensor_msg->header.stamp;
    message.header.frame_id = get_parameter("global_frame_id").as_string();
    const auto& [base_pose_in_map, base_pose_covariance] = new_estimate.value();
    tf2::toMsg(base_pose_in_map, message.pose.pose);
    tf2::covarianceEigenToRowMajor(base_pose_covariance, message.pose.covariance);
    pose_pub_->publish(message);
  }
}

void AmclNode::do_initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
  auto pose = Sophus::SE2d{};
  tf2::convert(message->pose.pose, pose);

  auto covariance = Eigen::Matrix3d{};
  tf2::covarianceRowMajorToEigen(message->pose.covariance, covariance);

  last_known_estimate_ = std::make_pair(pose, covariance);
  last_known_odom_transform_in_map_.reset();
  initialize_from_estimate(last_known_estimate_.value());
}

void AmclNode::global_localization_callback(
    [[maybe_unused]] std::shared_ptr<rmw_request_id_t> request_header,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Request> req,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> res) {
  initialize_from_map();
}

void AmclNode::nomotion_update_callback(
    [[maybe_unused]] std::shared_ptr<rmw_request_id_t> request_header,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Request> req,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> res) {
  if (!particle_filter_) {
    RCLCPP_WARN(get_logger(), "Ignoring no-motion update request because the particle filter has not been initialized");
    return;
  }

  particle_filter_->force_update();
  RCLCPP_INFO(get_logger(), "No-motion update requested");
}

bool AmclNode::initialize_from_estimate(const std::pair<Sophus::SE2d, Eigen::Matrix3d>& estimate) {
  RCLCPP_INFO(get_logger(), "Initializing particles from estimated pose and covariance");

  if (!particle_filter_) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particles: The particle filter has not been initialized");
    return false;
  }

  const auto& [pose, covariance] = estimate;

  try {
    particle_filter_->initialize(pose, covariance);
  } catch (const std::runtime_error& error) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particles: %s", error.what());
    return false;
  }

  enable_tf_broadcast_ = true;

  RCLCPP_INFO(
      get_logger(), "Particle filter initialized with %ld particles about initial pose x=%g, y=%g, yaw=%g",
      particle_filter_->particles().size(), pose.translation().x(), pose.translation().y(), pose.so2().log());

  return true;
}

bool AmclNode::initialize_from_map() {
  RCLCPP_INFO(get_logger(), "Initializing particles from map");

  if (!particle_filter_) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particles: The particle filter has not been initialized");
    return false;
  }

  particle_filter_->initialize_from_map();
  enable_tf_broadcast_ = true;

  RCLCPP_INFO(
      get_logger(), "Particle filter initialized with %ld particles distributed across the map",
      particle_filter_->particles().size());

  return true;
}

}  // namespace beluga_amcl

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(beluga_amcl::AmclNode)

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

#include <beluga_amcl/amcl_node.hpp>
#include <beluga_amcl/ros2_common.hpp>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/create_timer_ros.h>

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <range/v3/algorithm/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <beluga_ros/tf2_sophus.hpp>

namespace beluga_amcl {

namespace {

constexpr std::string_view kLikelihoodFieldModelName = "likelihood_field";
constexpr std::string_view kBeamSensorModelName = "beam";

}  // namespace

AmclNode::AmclNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode{"amcl", "", options} {
  RCLCPP_INFO(get_logger(), "Creating");
  declare_common_params(*this);
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

  if (get_parameter("autostart").as_bool()) {
    auto autostart_delay = std::chrono::duration<double>(get_parameter("autostart_delay").as_double());
    autostart_timer_ = create_wall_timer(autostart_delay, std::bind(&AmclNode::autostart_callback, this));
  }
}

AmclNode::~AmclNode() {
  RCLCPP_INFO(get_logger(), "Destroying");
  // In case this lifecycle node wasn't properly shut down, do it here
  on_shutdown(get_current_state());
}

void AmclNode::autostart_callback() {
  using lifecycle_msgs::msg::State;
  auto current_state = configure();
  if (current_state.id() != State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_WARN(get_logger(), "Failed to auto configure, shutting down");
    shutdown();
  }
  RCLCPP_WARN(get_logger(), "Auto configured successfully");
  current_state = activate();
  if (current_state.id() != State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_logger(), "Failed to auto activate, shutting down");
    shutdown();
  }
  RCLCPP_INFO(get_logger(), "Auto activated successfully");
  autostart_timer_->cancel();
}

AmclNode::CallbackReturn AmclNode::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Configuring");
  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>("particle_cloud", rclcpp::SensorDataQoS());
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", rclcpp::SystemDefaultsQoS());
  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Activating");
  particle_cloud_pub_->on_activate();
  pose_pub_->on_activate();

  {
    bond_ = std::make_unique<bond::Bond>("bond", get_name(), shared_from_this());
    bond_->setHeartbeatPeriod(0.10);
    bond_->setHeartbeatTimeout(4.0);
    bond_->start();
    RCLCPP_INFO(get_logger(), "Created bond (%s) to lifecycle manager", get_name());
  }

  // Accessing the particle filter is not thread safe.
  // This ensures that different callbacks are not called concurrently.
  auto common_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto common_subscription_options = rclcpp::SubscriptionOptions{};
  common_subscription_options.callback_group = common_callback_group;

  {
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(200ms, std::bind(&AmclNode::timer_callback, this), common_callback_group);
  }

  {
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        get_parameter("map_topic").as_string(), rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&AmclNode::map_callback, this, std::placeholders::_1), common_subscription_options);
    RCLCPP_INFO(get_logger(), "Subscribed to map_topic: %s", map_sub_->get_topic_name());
  }

  {
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        get_parameter("initial_pose_topic").as_string(), rclcpp::SystemDefaultsQoS(),
        std::bind(&AmclNode::initial_pose_callback, this, std::placeholders::_1), common_subscription_options);
    RCLCPP_INFO(get_logger(), "Subscribed to initial_pose_topic: %s", initial_pose_sub_->get_topic_name());
  }

  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface()));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
        *tf_buffer_, this,
        false);  // avoid using dedicated tf thread

    using LaserScanSubscriber =
        message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>;
    laser_scan_sub_ = std::make_unique<LaserScanSubscriber>(
        shared_from_this(), get_parameter("scan_topic").as_string(), rmw_qos_profile_sensor_data,
        common_subscription_options);

    // Message filter that caches laser scan readings until it is possible to transform
    // from laser frame to odom frame and update the particle filter.
    laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *laser_scan_sub_, *tf_buffer_, get_parameter("odom_frame_id").as_string(), 10, get_node_logging_interface(),
        get_node_clock_interface(), tf2::durationFromSec(get_parameter("transform_tolerance").as_double()));

    laser_scan_connection_ =
        laser_scan_filter_->registerCallback(std::bind(&AmclNode::laser_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to scan_topic: %s", laser_scan_sub_->getTopic().c_str());
  }

  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    // Ignore deprecated declaration warning to support Humble.
    // Message: use rclcpp::QoS instead of rmw_qos_profile_t
    global_localization_server_ = create_service<std_srvs::srv::Empty>(
        "reinitialize_global_localization",
        std::bind(
            &AmclNode::global_localization_callback, this, std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3),
        rmw_qos_profile_services_default, common_callback_group);
    RCLCPP_INFO(get_logger(), "Created reinitialize_global_localization service");

    nomotion_update_server_ = create_service<std_srvs::srv::Empty>(
        "request_nomotion_update",
        std::bind(
            &AmclNode::nomotion_update_callback, this, std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3),
        rmw_qos_profile_services_default, common_callback_group);
    RCLCPP_INFO(get_logger(), "Created request_nomotion_update service");

#pragma GCC diagnostic pop
  }

  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Deactivating");
  particle_cloud_pub_->on_deactivate();
  pose_pub_->on_deactivate();
  map_sub_.reset();
  initial_pose_sub_.reset();
  laser_scan_connection_.disconnect();
  laser_scan_filter_.reset();
  laser_scan_sub_.reset();
  tf_listener_.reset();
  tf_broadcaster_.reset();
  tf_buffer_.reset();
  global_localization_server_.reset();
  bond_.reset();
  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Cleaning up");
  particle_cloud_pub_.reset();
  pose_pub_.reset();
  particle_filter_.reset();
  enable_tf_broadcast_ = false;
  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(state);
    on_cleanup(state);
  }
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    on_cleanup(state);
  }
  return CallbackReturn::SUCCESS;
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
    return beluga::LikelihoodFieldModel{params, beluga_ros::OccupancyGrid{map}};
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

auto AmclNode::get_execution_policy(std::string_view name) -> beluga_ros::Amcl::execution_policy_variant {
  if (name == "seq") {
    return std::execution::seq;
  }
  if (name == "par") {
    return std::execution::par;
  }
  throw std::invalid_argument("Execution policy must be seq or par.");
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
      get_execution_policy(get_parameter("execution_policy").as_string()));
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
      particle_filter_ = make_particle_filter(std::move(map));
    } catch (const std::invalid_argument& error) {
      RCLCPP_ERROR(get_logger(), "Could not initialize particle filter: %s", error.what());
      return;
    }
  } else {
    particle_filter_->update_map(beluga_ros::OccupancyGrid{std::move(map)});
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

void AmclNode::timer_callback() {
  if (!particle_filter_) {
    return;
  }

  if (particle_cloud_pub_->get_subscription_count() == 0) {
    return;
  }

  particle_cloud_pub_->publish(
      make_representative_particle_cloud(*particle_filter_, get_parameter("global_frame_id").as_string(), now()));
}

void AmclNode::laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
  if (!particle_filter_) {
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
  const auto new_estimate = particle_filter_->update(
      base_pose_in_odom,  //
      beluga_ros::LaserScan{
          laser_scan,
          laser_pose_in_base,
          static_cast<std::size_t>(get_parameter("max_beams").as_int()),
          get_parameter("laser_min_range").as_double(),
          get_parameter("laser_max_range").as_double(),
      });
  const auto update_stop_time = std::chrono::high_resolution_clock::now();
  const auto update_duration = update_stop_time - update_start_time;

  if (new_estimate.has_value()) {
    const auto& [base_pose_in_map, _] = new_estimate.value();
    last_known_odom_transform_in_map_ = base_pose_in_map * base_pose_in_odom.inverse();
    last_known_estimate_ = new_estimate;

    RCLCPP_INFO(
        get_logger(), "Particle filter update iteration stats: %ld particles %ld points - %.3fms",
        particle_filter_->particles().size(), laser_scan->ranges.size(),
        std::chrono::duration<double, std::milli>(update_duration).count());
  }

  if (!last_known_estimate_.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Estimate not available for publishing");
    return;
  }

  const auto& [base_pose_in_map, base_pose_covariance] = last_known_estimate_.value();

  // New pose messages are only published on updates to the filter.
  if (new_estimate.has_value()) {
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped{};
    message.header.stamp = laser_scan->header.stamp;
    message.header.frame_id = get_parameter("global_frame_id").as_string();
    tf2::toMsg(base_pose_in_map, message.pose.pose);
    tf2::covarianceEigenToRowMajor(base_pose_covariance, message.pose.covariance);
    pose_pub_->publish(message);
  }

  // Transforms are always published to keep them current.
  if (enable_tf_broadcast_ && get_parameter("tf_broadcast").as_bool()) {
    auto message = geometry_msgs::msg::TransformStamped{};
    // Sending a transform that is valid into the future so that odom can be used.
    const auto expiration_stamp = tf2_ros::fromMsg(laser_scan->header.stamp) +
                                  tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
    message.header.stamp = tf2_ros::toMsg(expiration_stamp);
    message.header.frame_id = get_parameter("global_frame_id").as_string();
    message.child_frame_id = get_parameter("odom_frame_id").as_string();
    message.transform = tf2::toMsg(*last_known_odom_transform_in_map_);
    tf_broadcaster_->sendTransform(message);
  }
}

void AmclNode::initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
  const auto global_frame_id = get_parameter("global_frame_id").as_string();
  if (message->header.frame_id != global_frame_id) {
    RCLCPP_WARN(
        get_logger(), "Ignoring initial pose in frame \"%s\"; it must be in the global frame \"%s\"",
        message->header.frame_id.c_str(), global_frame_id.c_str());
    return;
  }

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

RCLCPP_COMPONENTS_REGISTER_NODE(beluga_amcl::AmclNode)

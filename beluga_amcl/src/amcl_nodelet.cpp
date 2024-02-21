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

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/SetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>

#include <optional>
#include <range/v3/algorithm/transform.hpp>

#include <beluga_amcl/amcl_nodelet.hpp>

#include <beluga_ros/laser_scan.hpp>
#include <beluga_ros/occupancy_grid.hpp>
#include <beluga_ros/tf2_sophus.hpp>

// LCOV_EXCL_BR_START: Disable branch coverage.

namespace beluga_amcl {

namespace {

constexpr std::string_view kDifferentialModelName = "differential_drive";
constexpr std::string_view kOmnidirectionalModelName = "omnidirectional_drive";
constexpr std::string_view kStationaryModelName = "stationary";

constexpr std::string_view kLikelihoodFieldModelName = "likelihood_field";
constexpr std::string_view kBeamSensorModelName = "beam";

constexpr std::string_view kAMCLDifferentialModelName = "diff-corrected";
constexpr std::string_view kAMCLOmnidirectionalModelName = "omni-corrected";

}  // namespace

void AmclNodelet::onInit() {
  ros::NodeHandle nh = getNodeHandle();

  config_server_ = std::make_unique<AmclConfigServer>(getPrivateNodeHandle());
  config_server_->setCallback(boost::bind(&AmclNodelet::config_callback, this, _1, _2));

  particle_cloud_timer_ = nh.createTimer(ros::Duration(0.2), &AmclNodelet::particle_cloud_timer_callback, this);
  particle_cloud_pub_ = nh.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);

  if (config_.use_map_topic) {
    map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>(config_.map_topic, 1, &AmclNodelet::map_callback, this);
    NODELET_INFO("Subscribed to map_topic: %s", map_sub_.getTopic().c_str());
  } else {
    get_map_client_ = nh.serviceClient<nav_msgs::GetMap>(config_.map_service);
    map_timer_ = nh.createTimer(ros::Duration(0.5), &AmclNodelet::map_timer_callback, this);
    NODELET_INFO("Connected to map_service: %s", get_map_client_.getService().c_str());
  }

  initial_pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      config_.initial_pose_topic, 1, &AmclNodelet::initial_pose_callback, this);
  NODELET_INFO("Subscribed to initial_pose_topic: %s", initial_pose_sub_.getTopic().c_str());

  if (config_.save_pose_rate > 0.0) {
    save_pose_timer_ =
        nh.createTimer(ros::Duration(1.0 / config_.save_pose_rate), &AmclNodelet::save_pose_timer_callback, this);
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  laser_scan_sub_.subscribe(nh, config_.scan_topic, 10);
  // Message filter that caches laser scan readings until it is possible to transform
  // from laser frame to odom frame and update the particle filter.
  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(
      laser_scan_sub_, *tf_buffer_, config_.odom_frame_id, 50, nh);
  laser_scan_connection_ = laser_scan_filter_->registerCallback(&AmclNodelet::laser_callback, this);

  NODELET_INFO("Subscribed to scan_topic: %s", laser_scan_sub_.getSubscriber().getTopic().c_str());

  global_localization_server_ =
      nh.advertiseService("global_localization", &AmclNodelet::global_localization_callback, this);
  NODELET_INFO("Created global_localization service");

  nomotion_update_server_ =
      nh.advertiseService("request_nomotion_update", &AmclNodelet::nomotion_update_callback, this);
  NODELET_INFO("Created request_nomotion_update service");

  set_map_server_ = nh.advertiseService("set_map", &AmclNodelet::set_map_callback, this);
  NODELET_INFO("Created set_map service");

  diagnosic_updater_.setHardwareID("None");
  diagnosic_updater_.add("Standard deviation", this, &AmclNodelet::update_covariance_diagnostics);
}

void AmclNodelet::config_callback(beluga_amcl::AmclConfig& config, [[maybe_unused]] uint32_t) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    /// Keep default configuration
    default_config_ = config;
  }

  if (config.restore_defaults) {
    config = default_config_;
    config.restore_defaults = false;
  }

  if (configured_) {
    /// Enforce read-only parameters once initialized
    config.map_topic = config_.map_topic;
    config.use_map_topic = config_.use_map_topic;
    config.map_service = config_.map_service;
    config.initial_pose_topic = config_.initial_pose_topic;
    config.scan_topic = config_.scan_topic;
    config.execution_policy = config_.execution_policy;
  }

  configured_ = true;
  config_ = config;

  if (last_known_estimate_.has_value()) {
    initialize_from_estimate(last_known_estimate_.value());
  }
}

auto AmclNodelet::get_initial_estimate() -> std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> {
  if (!config_.set_initial_pose) {
    return std::nullopt;
  }

  const auto pose = Sophus::SE2d{
      Sophus::SO2d{config_.initial_pose_a},
      Eigen::Vector2d{config_.initial_pose_x, config_.initial_pose_y},
  };

  Eigen::Matrix3d covariance;
  covariance.coeffRef(0, 0) = config_.initial_cov_xx;
  covariance.coeffRef(1, 1) = config_.initial_cov_yy;
  covariance.coeffRef(2, 2) = config_.initial_cov_aa;
  covariance.coeffRef(0, 1) = config_.initial_cov_xy;
  covariance.coeffRef(1, 0) = covariance.coeffRef(0, 1);
  covariance.coeffRef(0, 2) = config_.initial_cov_xa;
  covariance.coeffRef(2, 0) = covariance.coeffRef(0, 2);
  covariance.coeffRef(1, 2) = config_.initial_cov_ya;
  covariance.coeffRef(2, 1) = covariance.coeffRef(1, 2);

  return std::make_pair(pose, covariance);
}

auto AmclNodelet::get_motion_model(std::string_view name) -> beluga_ros::Amcl::motion_model_variant {
  if (name == kDifferentialModelName || name == kAMCLDifferentialModelName) {
    auto params = beluga::DifferentialDriveModelParam{};
    params.rotation_noise_from_rotation = config_.odom_alpha1;
    params.rotation_noise_from_translation = config_.odom_alpha2;
    params.translation_noise_from_translation = config_.odom_alpha3;
    params.translation_noise_from_rotation = config_.odom_alpha4;
    return beluga::DifferentialDriveModel{params};
  }
  if (name == kOmnidirectionalModelName || name == kAMCLOmnidirectionalModelName) {
    auto params = beluga::OmnidirectionalDriveModelParam{};
    params.rotation_noise_from_rotation = config_.odom_alpha1;
    params.rotation_noise_from_translation = config_.odom_alpha2;
    params.translation_noise_from_translation = config_.odom_alpha3;
    params.translation_noise_from_rotation = config_.odom_alpha4;
    params.strafe_noise_from_translation = config_.odom_alpha5;
    return beluga::OmnidirectionalDriveModel{params};
  }
  if (name == kStationaryModelName) {
    return beluga::StationaryModel{};
  }
  throw std::invalid_argument(std::string("Invalid motion model: ") + std::string(name));
}

auto AmclNodelet::get_sensor_model(std::string_view name, const nav_msgs::OccupancyGrid::ConstPtr& map)
    -> beluga_ros::Amcl::sensor_model_variant {
  if (name == kLikelihoodFieldModelName) {
    auto params = beluga::LikelihoodFieldModelParam{};
    params.max_obstacle_distance = config_.laser_likelihood_max_dist;
    params.max_laser_distance = config_.laser_max_range;
    params.z_hit = config_.laser_z_hit;
    params.z_random = config_.laser_z_rand;
    params.sigma_hit = config_.laser_sigma_hit;
    return beluga::LikelihoodFieldModel{params, beluga_ros::OccupancyGrid{map}};
  }
  if (name == kBeamSensorModelName) {
    auto params = beluga::BeamModelParam{};
    params.z_hit = config_.laser_z_hit;
    params.z_short = config_.laser_z_short;
    params.z_max = config_.laser_z_max;
    params.z_rand = config_.laser_z_rand;
    params.sigma_hit = config_.laser_sigma_hit;
    params.lambda_short = config_.laser_lambda_short;
    params.beam_max_range = config_.laser_max_range;
    return beluga::BeamSensorModel{params, beluga_ros::OccupancyGrid{map}};
  }
  throw std::invalid_argument(std::string("Invalid sensor model: ") + std::string(name));
}

auto AmclNodelet::get_execution_policy(std::string_view name) -> beluga_ros::Amcl::execution_policy_variant {
  if (name == "seq") {
    return std::execution::seq;
  }
  if (name == "par") {
    return std::execution::par;
  }
  throw std::invalid_argument("Execution policy must be seq or par.");
}

auto AmclNodelet::make_particle_filter(const nav_msgs::OccupancyGrid::ConstPtr& map)
    -> std::unique_ptr<beluga_ros::Amcl> {
  auto params = beluga_ros::AmclParams{};
  params.update_min_d = config_.update_min_d;
  params.update_min_a = config_.update_min_a;
  params.resample_interval = static_cast<std::size_t>(config_.resample_interval);
  params.selective_resampling = config_.selective_resampling;
  params.min_particles = static_cast<std::size_t>(config_.min_particles);
  params.max_particles = static_cast<std::size_t>(config_.max_particles);
  params.alpha_slow = config_.recovery_alpha_slow;
  params.alpha_fast = config_.recovery_alpha_fast;
  params.kld_epsilon = config_.kld_err;
  params.kld_z = config_.kld_z;
  params.spatial_resolution_x = config_.spatial_resolution_x;
  params.spatial_resolution_y = config_.spatial_resolution_y;
  params.spatial_resolution_theta = config_.spatial_resolution_theta;

  try {
    return std::make_unique<beluga_ros::Amcl>(
        beluga_ros::OccupancyGrid{map},                   //
        get_motion_model(config_.odom_model_type),        //
        get_sensor_model(config_.laser_model_type, map),  //
        params,                                           //
        get_execution_policy(config_.execution_policy));
  } catch (const std::invalid_argument& error) {
    NODELET_ERROR("Could not initialize particle filter: %s", error.what());
  }

  return nullptr;
}

void AmclNodelet::map_timer_callback(const ros::TimerEvent&) {
  if (!get_map_client_.exists()) {
    NODELET_INFO_THROTTLE(10, "Waiting for map service to be available");
    return;
  }
  const nav_msgs::GetMap::Request request;
  nav_msgs::GetMap::Response response;
  if (!get_map_client_.call(request, response)) {
    NODELET_WARN_THROTTLE(10, "Failed map request");
    return;
  }

  auto map = boost::make_shared<nav_msgs::OccupancyGrid>(response.map);
  handle_map_with_default_initial_pose(std::move(map));

  map_timer_.stop();
}

bool AmclNodelet::set_map_callback(nav_msgs::SetMap::Request& request, nav_msgs::SetMap::Response& response) {
  std::lock_guard<std::mutex> lock(mutex_);
  NODELET_INFO("A new map has been requested to be set");

  if (!particle_filter_) {
    NODELET_WARN("Ignoring set map request because the particle filter has not been initialized");
    response.success = static_cast<unsigned char>(false);
    return true;
  }

  if (request.map.header.frame_id != config_.global_frame_id) {
    NODELET_WARN(
        "Map frame \"%s\" doesn't match global frame \"%s\".", request.map.header.frame_id.c_str(),
        config_.global_frame_id.c_str());
  }

  if (request.initial_pose.header.frame_id != config_.global_frame_id) {
    NODELET_WARN(
        "Ignoring initial pose in frame \"%s\"; it must be in the global frame \"%s\".",
        request.initial_pose.header.frame_id.c_str(), config_.global_frame_id.c_str());
    response.success = static_cast<unsigned char>(false);
    return true;
  }

  auto map = boost::make_shared<nav_msgs::OccupancyGrid>(request.map);

  if (!particle_filter_) {
    particle_filter_ = make_particle_filter(map);
  } else {
    particle_filter_->update_map(beluga_ros::OccupancyGrid{map});
  }

  if (!particle_filter_) {
    return false;  // Initialization failed and the error was already logged.
  }

  last_known_map_ = map;

  auto pose = Sophus::SE2d{};
  tf2::convert(request.initial_pose.pose.pose, pose);

  auto covariance = Eigen::Matrix3d{};
  tf2::covarianceRowMajorToEigen(request.initial_pose.pose.covariance, covariance);

  last_known_estimate_ = std::make_pair(pose, covariance);
  const bool success = initialize_from_estimate(last_known_estimate_.value());

  response.success = static_cast<unsigned char>(success);
  return success;
}

void AmclNodelet::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  NODELET_INFO("A new map was received");

  if (particle_filter_ && config_.first_map_only) {
    NODELET_INFO("Ignoring new map because the particle filter has already been initialized.");
    return;
  }

  handle_map_with_default_initial_pose(message);
}

void AmclNodelet::handle_map_with_default_initial_pose(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  if (map->header.frame_id != config_.global_frame_id) {
    NODELET_WARN(
        "Map frame \"%s\" doesn't match global frame \"%s\".", map->header.frame_id.c_str(),
        config_.global_frame_id.c_str());
  }

  const bool should_reset_initial_pose = config_.always_reset_initial_pose ||  //
                                         (!particle_filter_ && !last_known_estimate_.has_value());

  if (!particle_filter_) {
    particle_filter_ = make_particle_filter(map);
  } else {
    particle_filter_->update_map(beluga_ros::OccupancyGrid{std::move(map)});
  }

  if (!particle_filter_) {
    return;  // Initialization failed and the error was already logged.
  }

  if (should_reset_initial_pose) {
    const auto initial_estimate = get_initial_estimate();
    if (initial_estimate.has_value()) {
      last_known_estimate_ = initial_estimate;
    }
  }

  if (last_known_estimate_.has_value()) {
    initialize_from_estimate(last_known_estimate_.value());
  } else {
    initialize_from_map();
  }

  last_known_map_ = map;
}

void AmclNodelet::particle_cloud_timer_callback(const ros::TimerEvent& ev) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!particle_filter_) {
    return;
  }

  if (particle_cloud_pub_.getNumSubscribers() == 0) {
    return;
  }

  auto message = geometry_msgs::PoseArray{};
  message.header.stamp = ev.current_real;
  message.header.frame_id = config_.global_frame_id;
  message.poses.resize(particle_filter_->particles().size());
  ranges::transform(
      particle_filter_->particles() | beluga::views::states,  //
      std::begin(message.poses), [](const auto& state) {
        auto message = geometry_msgs::Pose{};
        tf2::toMsg(state, message);
        return message;
      });
  particle_cloud_pub_.publish(message);
}

void AmclNodelet::laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!particle_filter_) {
    NODELET_WARN_THROTTLE(2, "Ignoring laser data because the particle filter has not been initialized");
    return;
  }

  auto base_pose_in_odom = Sophus::SE2d{};
  try {
    // Use the lookupTransform overload with no timeout since we're not using a dedicated
    // tf thread. The message filter we are using avoids the need for it.
    tf2::convert(
        tf_buffer_->lookupTransform(config_.odom_frame_id, config_.base_frame_id, laser_scan->header.stamp).transform,
        base_pose_in_odom);
  } catch (const tf2::TransformException& error) {
    NODELET_ERROR("Could not transform from odom to base: %s", error.what());
    return;
  }

  auto laser_pose_in_base = Sophus::SE3d{};
  try {
    tf2::convert(
        tf_buffer_->lookupTransform(config_.base_frame_id, laser_scan->header.frame_id, laser_scan->header.stamp)
            .transform,
        laser_pose_in_base);
  } catch (const tf2::TransformException& error) {
    NODELET_ERROR("Could not transform from base to laser: %s", error.what());
    return;
  }

  const auto update_start_time = std::chrono::high_resolution_clock::now();
  const auto new_estimate = particle_filter_->update(
      base_pose_in_odom,  //
      beluga_ros::LaserScan{
          laser_scan,
          laser_pose_in_base,
          static_cast<std::size_t>(config_.laser_max_beams),
          config_.laser_min_range,
          config_.laser_max_range,
      });
  const auto update_stop_time = std::chrono::high_resolution_clock::now();
  const auto update_duration = update_stop_time - update_start_time;

  if (new_estimate.has_value()) {
    last_known_estimate_ = new_estimate;

    NODELET_INFO(
        "Particle filter update iteration stats: %ld particles %ld points - %.3fms",
        particle_filter_->particles().size(),  //
        laser_scan->ranges.size(),             //
        std::chrono::duration<double, std::milli>(update_duration).count());
  }

  if (!last_known_estimate_.has_value()) {
    NODELET_WARN_THROTTLE(2, "Estimate not available for publishing");
    return;
  }

  const auto& [base_pose_in_map, base_pose_covariance] = last_known_estimate_.value();

  // New pose messages are only published on updates to the filter.
  if (new_estimate.has_value()) {
    auto message = geometry_msgs::PoseWithCovarianceStamped{};
    message.header.stamp = laser_scan->header.stamp;
    message.header.frame_id = config_.global_frame_id;
    tf2::toMsg(base_pose_in_map, message.pose.pose);
    tf2::covarianceEigenToRowMajor(base_pose_covariance, message.pose.covariance);
    pose_pub_.publish(message);
  }

  // Transforms are always published to keep them current
  if (enable_tf_broadcast_ && config_.tf_broadcast) {
    const auto odom_transform_in_map = base_pose_in_map * base_pose_in_odom.inverse();
    auto message = geometry_msgs::TransformStamped{};
    // Sending a transform that is valid into the future so that odom can be used.
    message.header.stamp = ros::Time::now() + ros::Duration(config_.transform_tolerance);
    message.header.frame_id = config_.global_frame_id;
    message.child_frame_id = config_.odom_frame_id;
    message.transform = tf2::toMsg(odom_transform_in_map);
    tf_broadcaster_->sendTransform(message);
  }

  diagnosic_updater_.update();
}

void AmclNodelet::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!particle_filter_) {
    NODELET_WARN("Ignoring initial pose request because the particle filter has not been initialized.");
    return;
  }

  if (message->header.frame_id != config_.global_frame_id) {
    NODELET_WARN(
        "Ignoring initial pose in frame \"%s\"; it must be in the global frame \"%s\".",
        message->header.frame_id.c_str(), config_.global_frame_id.c_str());
    return;
  }

  auto pose = Sophus::SE2d{};
  tf2::convert(message->pose.pose, pose);

  auto covariance = Eigen::Matrix3d{};
  tf2::covarianceRowMajorToEigen(message->pose.covariance, covariance);

  last_known_estimate_ = std::make_pair(pose, covariance);
  initialize_from_estimate(last_known_estimate_.value());
}

void AmclNodelet::save_pose_timer_callback(const ros::TimerEvent&) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_known_estimate_.has_value()) {
    const auto& [pose, covariance] = last_known_estimate_.value();
    config_.initial_pose_x = pose.translation().x();
    config_.initial_pose_y = pose.translation().y();
    config_.initial_pose_a = pose.so2().log();
    config_.initial_cov_xx = covariance(0, 0);
    config_.initial_cov_yy = covariance(1, 1);
    config_.initial_cov_aa = covariance(2, 2);
    config_.initial_cov_xy = covariance(0, 1);
    config_.initial_cov_xa = covariance(0, 2);
    config_.initial_cov_ya = covariance(1, 2);
    config_server_->updateConfig(config_);
  }
}

bool AmclNodelet::global_localization_callback(
    [[maybe_unused]] std_srvs::Empty::Request&,
    [[maybe_unused]] std_srvs::Empty::Response&) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialize_from_map()) {
    enable_tf_broadcast_ = true;
    return true;
  }
  return false;
}

bool AmclNodelet::nomotion_update_callback(
    [[maybe_unused]] std_srvs::Empty::Request&,
    [[maybe_unused]] std_srvs::Empty::Response&) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!particle_filter_) {
    NODELET_WARN("Ignoring no-motion request because the particle filter has not been initialized.");
    return false;
  }

  particle_filter_->force_update();
  NODELET_INFO("No-motion update requested");
  return true;
}

bool AmclNodelet::initialize_from_estimate(const std::pair<Sophus::SE2d, Eigen::Matrix3d>& estimate) {
  NODELET_INFO("Initializing particles from estimated pose and covariance");

  if (!particle_filter_) {
    NODELET_ERROR("Could not initialize particles: The particle filter has not been initialized");
    return false;
  }

  const auto& [pose, covariance] = estimate;

  try {
    particle_filter_->initialize(pose, covariance);
  } catch (const std::runtime_error& error) {
    NODELET_ERROR("Could not initialize particles: %s", error.what());
    return false;
  }

  enable_tf_broadcast_ = true;

  NODELET_INFO(
      "Particle filter initialized with %ld particles about initial pose x=%g, y=%g, yaw=%g",
      particle_filter_->particles().size(),  //
      pose.translation().x(),                //
      pose.translation().y(),                //
      pose.so2().log());

  return true;
}

bool AmclNodelet::initialize_from_map() {
  NODELET_INFO("Initializing particles from map");

  if (!particle_filter_) {
    NODELET_ERROR("Could not initialize particles: The particle filter has not been initialized");
    return false;
  }

  particle_filter_->initialize_from_map();

  // NOTE: We do not set `enable_tf_broadcast_ = true` here to match the original implementation.

  NODELET_INFO(
      "Particle filter initialized with %ld particles distributed across the map",
      particle_filter_->particles().size());

  return true;
}

void AmclNodelet::update_covariance_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
  if (last_known_estimate_.has_value()) {
    const auto& [_, covariance] = last_known_estimate_.value();
    const double std_x = std::sqrt(covariance(0, 0));
    const double std_y = std::sqrt(covariance(1, 1));
    const double std_yaw = std::sqrt(covariance(2, 2));

    status.add("std_x", std_x);
    status.add("std_y", std_y);
    status.add("std_yaw", std_yaw);
    status.add("std_warn_level_x", config_.std_warn_level_x);
    status.add("std_warn_level_y", config_.std_warn_level_y);
    status.add("std_warn_level_yaw", config_.std_warn_level_yaw);

    if (std_x > config_.std_warn_level_x || std_y > config_.std_warn_level_y || std_yaw > config_.std_warn_level_yaw) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Too large");
      return;
    }
  }
  status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
}

}  // namespace beluga_amcl

PLUGINLIB_EXPORT_CLASS(beluga_amcl::AmclNodelet, nodelet::Nodelet)

// LCOV_EXCL_BR_STOP

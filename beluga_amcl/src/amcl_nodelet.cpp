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

#include <ros/ros.h>
#include <bondcpp/bond.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>

#include <range/v3/algorithm/transform.hpp>

#include "beluga_amcl/private/amcl_nodelet.hpp"

#include "beluga_amcl/amcl_node_utils.hpp"
#include "beluga_amcl/occupancy_grid.hpp"
#include "beluga_amcl/tf2_sophus.hpp"
#include "beluga_amcl/private/execution_policy.hpp"

// LCOV_EXCL_BR_START: Disable branch coverage.

namespace beluga_amcl
{

void AmclNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();

  config_server_ = std::make_unique<AmclConfigServer>(getPrivateNodeHandle());
  config_server_->setCallback(boost::bind(&AmclNodelet::config_callback, this, _1, _2));

  particle_cloud_pub_ = nh.advertise<geometry_msgs::PoseArray>("particle_cloud", 2);
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 2);

  bond_ = std::make_unique<bond::Bond>("bond", getName());
  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();
  NODELET_INFO("Created bond (%s)", getName().c_str());

  timer_ = nh.createTimer(ros::Duration(0.2), &AmclNodelet::timer_callback, this);

  map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>(
    config_.map_topic, 1, &AmclNodelet::map_callback, this);
  NODELET_INFO("Subscribed to map_topic: %s", map_sub_.getTopic().c_str());

  initial_pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
    config_.initial_pose_topic, 1, &AmclNodelet::initial_pose_callback, this);
  NODELET_INFO("Subscribed to initial_pose_topic: %s", initial_pose_sub_.getTopic().c_str());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  laser_scan_sub_.subscribe(nh, config_.scan_topic, 10);
  // Message filter that caches laser scan readings until it is possible to transform
  // from laser frame to odom frame and update the particle filter.
  laser_scan_filter_ =
    std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(
    laser_scan_sub_, *tf_buffer_, config_.odom_frame_id, 50, nh);

  using LaserCallback = boost::function<void (sensor_msgs::LaserScan::ConstPtr)>;
  execution::Policy execution_policy = std::execution::par;
  try {
    execution_policy = beluga_amcl::execution::policy_from_string(config_.execution_policy);
  } catch (const std::invalid_argument &) {
    NODELET_WARN_STREAM(
      "execution_policy param should be [seq, par], got: " <<
        config_.execution_policy << "\nUsing the default parallel policy.");
  }
  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::visit(
      [this](const auto & policy) -> LaserCallback
      {
        return [this, policy](const sensor_msgs::LaserScan::ConstPtr & laser_scan) {
          this->laser_callback(policy, laser_scan);
        };
      }, execution_policy));
  NODELET_INFO(
    "Subscribed to scan_topic: %s",
    laser_scan_sub_.getSubscriber().getTopic().c_str());

  global_localization_server_ = nh.advertiseService(
    "reinitialize_global_localization",
    &AmclNodelet::global_localization_callback, this);
  NODELET_INFO("Created reinitialize_global_localization service");
}

namespace
{

std::pair<Sophus::SE2d, Eigen::Matrix3d>
extract_initial_estimate(const beluga_amcl::AmclConfig & config)
{
  const auto pose = Sophus::SE2d{
    Sophus::SO2d{config.initial_pose_yaw},
    Eigen::Vector2d{config.initial_pose_x, config.initial_pose_y}};

  Eigen::Matrix3d covariance;
  covariance.coeffRef(0, 0) = config.initial_pose_covariance_x;
  covariance.coeffRef(1, 1) = config.initial_pose_covariance_y;
  covariance.coeffRef(2, 2) = config.initial_pose_covariance_yaw;
  covariance.coeffRef(0, 1) = config.initial_pose_covariance_xy;
  covariance.coeffRef(1, 0) = covariance.coeffRef(0, 1);
  covariance.coeffRef(0, 2) = config.initial_pose_covariance_xyaw;
  covariance.coeffRef(2, 0) = covariance.coeffRef(0, 2);
  covariance.coeffRef(1, 2) = config.initial_pose_covariance_yyaw;
  covariance.coeffRef(2, 1) = covariance.coeffRef(1, 2);

  return std::make_pair(pose, covariance);
}

}  // namespace

void AmclNodelet::config_callback(beluga_amcl::AmclConfig & config, [[maybe_unused]] uint32_t)
{
  std::lock_guard<std::mutex> lock(mutex_);
  /// Enforce read-only parameters once initialized
  if (laser_scan_filter_) {
    config.map_topic = config_.map_topic;
    config.initial_pose_topic = config_.initial_pose_topic;
    config.scan_topic = config_.scan_topic;
    config.execution_policy = config_.execution_policy;
  }
  config_ = config;

  if (particle_filter_) {
    particle_filter_ = make_particle_filter(last_known_map_);

    const bool should_reset_initial_pose =
      !last_known_estimate_.has_value() ||
      config_.always_reset_initial_pose;

    if (should_reset_initial_pose && config_.set_initial_pose) {
      const auto [pose, covariance] = extract_initial_estimate(config_);
      initialize_with_pose(pose, covariance);
    } else if (last_known_estimate_.has_value()) {
      const auto & [pose, covariance] = last_known_estimate_.value();
      initialize_with_pose(pose, covariance);
    }
  }
}

std::unique_ptr<LaserLocalizationInterface2d>
AmclNodelet::make_particle_filter(const nav_msgs::OccupancyGrid::ConstPtr & map)
{
  auto sampler_params = beluga::AdaptiveSamplerParam{};
  sampler_params.alpha_slow = config_.recovery_alpha_slow;
  sampler_params.alpha_fast = config_.recovery_alpha_fast;

  auto limiter_params = beluga::KldLimiterParam<Sophus::SE2d>{};
  limiter_params.min_samples = static_cast<std::size_t>(config_.min_particles);
  limiter_params.max_samples = static_cast<std::size_t>(config_.max_particles);
  limiter_params.spatial_hasher = beluga::spatial_hash<Sophus::SE2d>{
    config_.spatial_resolution_x,
    config_.spatial_resolution_y,
    config_.spatial_resolution_theta
  };
  limiter_params.kld_epsilon = config_.pf_err;
  limiter_params.kld_z = config_.pf_z;

  auto resample_on_motion_params = beluga::ResampleOnMotionPolicyParam{};
  resample_on_motion_params.update_min_d = config_.update_min_d;
  resample_on_motion_params.update_min_a = config_.update_min_a;

  auto resample_interval_params = beluga::ResampleIntervalPolicyParam{};
  resample_interval_params.resample_interval_count =
    static_cast<std::size_t>(config_.resample_interval);

  auto selective_resampling_params = beluga::SelectiveResamplingPolicyParam{};
  selective_resampling_params.enabled = config_.selective_resampling;

  auto get_motion_descriptor = [this](std::string_view name) -> MotionDescriptor {
      if (name == kDifferentialModelName || name == kNav2DifferentialModelName) {
        auto params = beluga::DifferentialDriveModelParam{};
        params.rotation_noise_from_rotation = config_.alpha1;
        params.rotation_noise_from_translation = config_.alpha2;
        params.translation_noise_from_translation = config_.alpha3;
        params.translation_noise_from_rotation = config_.alpha4;
        return DifferentialDrive{params};
      } else if (name == kOmnidirectionalModelName || name == kNav2OmnidirectionalModelName) {
        auto params = beluga::OmnidirectionalDriveModelParam{};
        params.rotation_noise_from_rotation = config_.alpha1;
        params.rotation_noise_from_translation = config_.alpha2;
        params.translation_noise_from_translation = config_.alpha3;
        params.translation_noise_from_rotation = config_.alpha4;
        params.strafe_noise_from_translation = config_.alpha5;
        return OmnidirectionalDrive{params};
      } else if (name == kStationaryModelName) {
        return Stationary{};
      }
      throw std::invalid_argument(std::string("Invalid motion model: ") + std::string(name));
    };

  auto get_sensor_descriptor = [this](std::string_view name) -> SensorDescriptor {
      if (name == kLikelihoodFieldModelName) {
        auto params = beluga::LikelihoodFieldModelParam{};
        params.max_obstacle_distance = config_.laser_likelihood_max_dist;
        params.max_laser_distance = config_.laser_max_range;
        params.z_hit = config_.z_hit;
        params.z_random = config_.z_rand;
        params.sigma_hit = config_.sigma_hit;
        return LikelihoodField{params};
      }
      if (name == kBeamSensorModelName) {
        auto params = beluga::BeamModelParam{};
        params.z_hit = config_.z_hit;
        params.z_short = config_.z_short;
        params.z_max = config_.z_max;
        params.z_rand = config_.z_rand;
        params.sigma_hit = config_.sigma_hit;
        params.lambda_short = config_.lambda_short;
        params.beam_max_range = config_.laser_max_range;
        return BeamSensorModel{params};
      }
      throw std::invalid_argument(std::string("Invalid sensor model: ") + std::string(name));
    };

  try {
    using beluga::mixin::make_mixin;
    return make_mixin<LaserLocalizationInterface2d, AdaptiveMonteCarloLocalization2d>(
      sampler_params,
      limiter_params,
      resample_on_motion_params,
      resample_interval_params,
      selective_resampling_params,
      get_motion_descriptor(config_.robot_model_type),
      get_sensor_descriptor(config_.laser_model_type),
      OccupancyGrid{map}
    );
  } catch (const std::invalid_argument & error) {
    NODELET_ERROR("Coudn't instantiate the particle filter: %s", error.what());
  }
  return nullptr;
}

void AmclNodelet::map_callback(const nav_msgs::OccupancyGrid::ConstPtr & map)
{
  std::lock_guard<std::mutex> lock(mutex_);
  NODELET_INFO("A new map was received");

  if (particle_filter_ && config_.first_map_only) {
    NODELET_INFO(
      "Ignoring new map because the particle filter"
      "has already been initialized.");
    return;
  }

  if (map->header.frame_id != config_.global_frame_id) {
    NODELET_WARN(
      "Map frame \"%s\" doesn't match global frame \"%s\".",
      map->header.frame_id.c_str(), config_.global_frame_id.c_str());
  }

  bool should_reset_initial_pose = config_.always_reset_initial_pose;
  if (!particle_filter_) {
    particle_filter_ = make_particle_filter(map);
    should_reset_initial_pose |= !last_known_estimate_.has_value();
  } else {
    particle_filter_->update_map(OccupancyGrid{map});
    should_reset_initial_pose = config_.always_reset_initial_pose;
  }

  if (should_reset_initial_pose && config_.set_initial_pose) {
    const auto [pose, covariance] = extract_initial_estimate(config_);
    initialize_with_pose(pose, covariance);
  } else if (last_known_estimate_.has_value()) {
    const auto & [pose, covariance] = last_known_estimate_.value();
    initialize_with_pose(pose, covariance);
  }

  last_known_map_ = map;
}

void AmclNodelet::timer_callback(const ros::TimerEvent & ev)
{
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
  message.poses.resize(particle_filter_->particle_count());
  ranges::transform(
    ranges::views::zip(particle_filter_->states_view(), particle_filter_->weights_view()),
    std::begin(message.poses), [](const auto & particle) {
      const auto & [state, _] = particle;
      auto message = geometry_msgs::Pose{};
      tf2::toMsg(state, message);
      return message;
    });
  particle_cloud_pub_.publish(message);
}

template<typename ExecutionPolicy>
void AmclNodelet::laser_callback(
  ExecutionPolicy && exec_policy,
  const sensor_msgs::LaserScan::ConstPtr & laser_scan)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!particle_filter_) {
    NODELET_WARN_THROTTLE(
      2, "Ignoring laser data because the particle filter has not been initialized");
    return;
  }

  auto odom_to_base_transform = Sophus::SE2d{};
  try {
    // Use the lookupTransform overload with no timeout since we're not using a dedicated
    // tf thread. The message filter we are using avoids the need for it.
    tf2::convert(
      tf_buffer_->lookupTransform(
        config_.odom_frame_id,
        config_.base_frame_id,
        laser_scan->header.stamp).transform,
      odom_to_base_transform);
  } catch (const tf2::TransformException & error) {
    NODELET_ERROR("Could not transform from odom to base: %s", error.what());
    return;
  }

  auto base_to_laser_transform = Sophus::SE3d{};
  try {
    tf2::convert(
      tf_buffer_->lookupTransform(
        config_.base_frame_id,
        laser_scan->header.frame_id,
        laser_scan->header.stamp).transform,
      base_to_laser_transform);
  } catch (const tf2::TransformException & error) {
    NODELET_ERROR("Could not transform from base to laser: %s", error.what());
    return;
  }

  {
    const auto time1 = std::chrono::high_resolution_clock::now();
    particle_filter_->update_motion(odom_to_base_transform);
    particle_filter_->sample(exec_policy);
    const auto time2 = std::chrono::high_resolution_clock::now();
    particle_filter_->update_sensor(
      utils::make_points_from_laser_scan(
        *laser_scan,
        base_to_laser_transform,
        static_cast<std::size_t>(config_.max_beams),
        static_cast<float>(config_.laser_min_range),
        static_cast<float>(config_.laser_max_range)));
    particle_filter_->reweight(exec_policy);
    const auto time3 = std::chrono::high_resolution_clock::now();
    particle_filter_->resample();
    const auto time4 = std::chrono::high_resolution_clock::now();

    NODELET_INFO_THROTTLE(
      2,
      "Particle filter update stats: %ld particles %ld points - %.3fms %.3fms %.3fms",
      particle_filter_->particle_count(),
      laser_scan->ranges.size(),
      std::chrono::duration<double, std::milli>(time2 - time1).count(),
      std::chrono::duration<double, std::milli>(time3 - time2).count(),
      std::chrono::duration<double, std::milli>(time4 - time3).count());
  }

  last_known_estimate_ = particle_filter_->estimate();
  const auto & [pose, covariance] = last_known_estimate_.value();

  {
    auto message = geometry_msgs::PoseWithCovarianceStamped{};
    message.header.stamp = laser_scan->header.stamp;
    message.header.frame_id = config_.global_frame_id;
    tf2::toMsg(pose, message.pose.pose);
    tf2::covarianceEigenToRowMajor(covariance, message.pose.covariance);
    pose_pub_.publish(message);
  }

  if (enable_tf_broadcast_ && config_.tf_broadcast) {
    auto message = geometry_msgs::TransformStamped{};
    // Sending a transform that is valid into the future so that odom can be used.
    message.header.stamp = ros::Time::now() + ros::Duration(config_.transform_tolerance);
    message.header.frame_id = config_.global_frame_id;
    message.child_frame_id = config_.odom_frame_id;
    message.transform = tf2::toMsg(odom_to_base_transform * pose.inverse());
    tf_broadcaster_->sendTransform(message);
  }
}

void AmclNodelet::initial_pose_callback(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!particle_filter_) {
    NODELET_WARN(
      "Ignoring initial pose request because the "
      "particle filter has not been initialized.");
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

  initialize_with_pose(pose, covariance);
}

void AmclNodelet::initialize_with_pose(
  const Sophus::SE2d & pose,
  const Eigen::Matrix3d & covariance)
{
  try {
    beluga_amcl::initialize_with_pose(
      pose, covariance, particle_filter_.get());
    enable_tf_broadcast_ = true;
    NODELET_INFO(
      "Particle filter initialized with %ld particles about "
      "initial pose x=%g, y=%g, yaw=%g", particle_filter_->particle_count(),
      pose.translation().x(), pose.translation().y(), pose.so2().log());
  } catch (const std::runtime_error & error) {
    NODELET_ERROR("Could not generate particles: %s", error.what());
  }
}

bool AmclNodelet::global_localization_callback(
  [[maybe_unused]] std_srvs::Empty::Request &,
  [[maybe_unused]] std_srvs::Empty::Response &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!particle_filter_) {
    NODELET_WARN(
      "Ignoring global localization request because "
      "the particle filter has not been initialized.");
    return false;
  }
  particle_filter_->reinitialize();
  NODELET_INFO("Global initialization done!");
  enable_tf_broadcast_ = true;
  return true;
}

}  // namespace beluga_amcl

PLUGINLIB_EXPORT_CLASS(beluga_amcl::AmclNodelet, nodelet::Nodelet)

// LCOV_EXCL_BR_STOP

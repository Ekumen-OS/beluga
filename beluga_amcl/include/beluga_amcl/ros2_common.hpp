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

#ifndef BELUGA_AMCL_ROS2_COMMON_HPP
#define BELUGA_AMCL_ROS2_COMMON_HPP

#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/msg/detail/particle_cloud__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sophus/se2.hpp>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/create_timer_ros.h>

#include <beluga_ros/tf2_sophus.hpp>
#include <string_view>

namespace beluga_amcl {

/// String identifier for a diff drive model.
constexpr std::string_view kDifferentialModelName = "differential_drive";
/// String identifier for a omnidirectional drive model.
constexpr std::string_view kOmnidirectionalModelName = "omnidirectional_drive";
/// String identifier for a stationary model.
constexpr std::string_view kStationaryModelName = "stationary";
/// String identifier for a diff drive model.
constexpr std::string_view kNav2DifferentialModelName = "nav2_amcl::DifferentialMotionModel";
/// String identifier for a omnidirectional model name.
constexpr std::string_view kNav2OmnidirectionalModelName = "nav2_amcl::OmniMotionModel";

/// Declares common AMCL-related parameters to the node.
inline void declare_common_params(rclcpp_lifecycle::LifecycleNode& node) {
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "The name of the coordinate frame published by the localization system.";
    node.declare_parameter("global_frame_id", rclcpp::ParameterValue("map"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "The name of the coordinate frame to use for odometry.";
    node.declare_parameter("odom_frame_id", rclcpp::ParameterValue("odom"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "The name of the coordinate frame to use for the robot base.";
    node.declare_parameter("base_frame_id", rclcpp::ParameterValue("base_footprint"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Path to load the map from an hdf5 file.";
    node.declare_parameter("map_path", rclcpp::ParameterValue("map_path"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Topic to subscribe to in order to receive the initial pose of the robot.";
    node.declare_parameter("initial_pose_topic", rclcpp::ParameterValue("initialpose"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Topic to subscribe to in order to receive the laser scan for localization.";
    node.declare_parameter("scan_topic", rclcpp::ParameterValue("scan"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Minimum allowed number of particles.";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    node.declare_parameter("min_particles", rclcpp::ParameterValue(500), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Maximum allowed number of particles.";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    node.declare_parameter("max_particles", rclcpp::ParameterValue(2000), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Exponential decay rate for the slow average weight filter, used in deciding when to recover "
        "by adding random poses.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1;
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("recovery_alpha_slow", rclcpp::ParameterValue(0.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Exponential decay rate for the fast average weight filter, used in deciding when to recover "
        "by adding random poses.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1;
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("recovery_alpha_fast", rclcpp::ParameterValue(0.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Maximum particle filter population error between the true distribution "
        "and the estimated distribution. It is used in KLD resampling to limit the "
        "allowed number of particles to the minimum necessary.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1;
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("pf_err", rclcpp::ParameterValue(0.05), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Upper standard normal quantile for P, where P is the probability "
        "that the error in the estimated distribution will be less than pf_err "
        "in KLD resampling.";
    node.declare_parameter("pf_z", rclcpp::ParameterValue(0.99), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Resolution in meters for the X axis used to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("spatial_resolution_x", rclcpp::ParameterValue(0.5), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Resolution in meters for the Y axis used to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("spatial_resolution_y", rclcpp::ParameterValue(0.5), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Resolution in radians for the theta axis to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 2 * Sophus::Constants<double>::pi();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter(
        "spatial_resolution_theta", rclcpp::ParameterValue(10 * Sophus::Constants<double>::pi() / 180), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Number of filter updates required before resampling. ";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 1;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    node.declare_parameter("resample_interval", rclcpp::ParameterValue(1), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "When set to true, will reduce the resampling rate when not needed and help "
        "avoid particle deprivation. The resampling will only happen if the effective "
        "number of particles (N_eff = 1/(sum(k_i^2))) is lower than half the current "
        "number of particles.";
    descriptor.read_only = true;
    node.declare_parameter("selective_resampling", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Set this to false to prevent amcl from publishing the transform "
        "between the global frame and the odometry frame.";
    node.declare_parameter("tf_broadcast", rclcpp::ParameterValue(true), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Time with which to post-date the transform that is published, "
        "to indicate that this transform is valid into the future";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("transform_tolerance", rclcpp::ParameterValue(1.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Which motion model to use [differential_drive, omnidirectional_drive, stationary].";
    node.declare_parameter("robot_model_type", rclcpp::ParameterValue(std::string(kDifferentialModelName)), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Rotation noise from rotation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("alpha1", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Rotation noise from translation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("alpha2", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Translation noise from translation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("alpha3", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Translation noise from rotation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("alpha4", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Strafe noise from translation for the omnidirectional drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("alpha5", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Rotational movement required before performing a filter update.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 2 * Sophus::Constants<double>::pi();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("update_min_a", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Translational movement required before performing a filter update.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("update_min_d", rclcpp::ParameterValue(0.25), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Maximum scan range to be considered.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("laser_max_range", rclcpp::ParameterValue(100.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Minimum scan range to be considered.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    node.declare_parameter("laser_min_range", rclcpp::ParameterValue(0.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "How many evenly-spaced beams in each scan will be used when updating the filter.";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 2;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    node.declare_parameter("max_beams", rclcpp::ParameterValue(60), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Set the initial pose from the initial_pose parameters.";
    node.declare_parameter("set_initial_pose", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose x axis coordinate.";
    node.declare_parameter("initial_pose.x", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose y axis coordinate.";
    node.declare_parameter("initial_pose.y", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yaw rotation.";
    node.declare_parameter("initial_pose.yaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose x axis covariance.";
    node.declare_parameter("initial_pose.covariance_x", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose y axis covariance.";
    node.declare_parameter("initial_pose.covariance_y", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yaw covariance.";
    node.declare_parameter("initial_pose.covariance_yaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose xy covariance.";
    node.declare_parameter("initial_pose.covariance_xy", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose xyaw covariance.";
    node.declare_parameter("initial_pose.covariance_xyaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yyaw covariance.";
    node.declare_parameter("initial_pose.covariance_yyaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Execution policy used to process particles [seq, par].";
    node.declare_parameter("execution_policy", "seq", descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Whether the node should configure and activate itself or not.";
    node.declare_parameter("autostart", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Delay, in seconds, before autostarting if autostarting.";
    node.declare_parameter("autostart_delay", 0.0, descriptor);
  }
}

/// Helper struct to hold a SE2 state and its weight.
struct RepresentativeData {
  /// SE2 state.
  Sophus::SE2d state;
  /// Weight.
  double weight{0.};
};

/// Hash implementation for SE2 transforms.
struct RepresentativeBinHash {
  /// Computes a hash out of a SE2 transform.
  std::size_t operator()(const Sophus::SE2d& s) const noexcept {
    std::size_t h1 = std::hash<double>{}(s.translation().x());
    std::size_t h2 = std::hash<double>{}(s.translation().y());
    std::size_t h3 = std::hash<double>{}(s.so2().log());
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

/// Equal comparison functor for SE2 transforms.
struct RepresentativeBinEqual {
  /// Actual hash implementation.
  bool operator()(const Sophus::SE2d& lhs, const Sophus::SE2d& rhs) const noexcept {
    // good enough, since copies of the same candidate are expected to be identical copies
    return lhs.translation().x() == rhs.translation().x() &&  //
           lhs.translation().y() == rhs.translation().y() &&  //
           lhs.so2().log() == rhs.so2().log();
  }
};

// Particle weights from the filter may or may not be representative of the
// true distribution. If we resampled, they are not, and there will be multiple copies
// of the most likely candidates, all with unit weight. In this case the number of copies
// is a proxy for the prob density at each candidate. If we did not resample before updating
// the estimation and publishing this message (which can happen if the resample interval
// is set to something other than 1), then all particles are expected to be different
// and their weights are proportional to the prob density at each candidate.
//
// Only the combination of both the state distribution and the candidate weights together
// provide information about the probability density at each candidate.
// To handle both cases, we group repeated candidates and compute the accumulated weight.
template <class ParticleFilter>
nav2_msgs::msg::ParticleCloud make_representative_particle_cloud(
    const ParticleFilter& particle_filter,
    std::string_view global_frame,
    rclcpp::Time now) {
  std::unordered_map<Sophus::SE2d, RepresentativeData, RepresentativeBinHash, RepresentativeBinEqual>
      representatives_map;
  representatives_map.reserve(particle_filter.particles().size());

  double max_weight = 1e-5;  // never risk dividing by zero

  for (const auto& [state, weight] : particle_filter.particles()) {
    auto& representative = representatives_map[state];  // if the element does not exist, create it
    representative.state = state;
    representative.weight += weight;
    if (representative.weight > max_weight) {
      max_weight = representative.weight;
    }
  }

  auto message = nav2_msgs::msg::ParticleCloud{};
  message.header.stamp = now;
  message.header.frame_id = global_frame;
  message.particles.reserve(particle_filter.particles().size());

  for (const auto& [key, representative] : representatives_map) {
    auto& particle = message.particles.emplace_back();
    tf2::toMsg(representative.state, particle.pose);
    particle.weight = representative.weight / max_weight;
  }

  return message;
}

}  // namespace beluga_amcl
#endif

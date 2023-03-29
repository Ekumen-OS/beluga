// Copyright 2022 Ekumen, Inc.
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

#include "beluga_amcl/private/amcl_node.hpp"

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/create_timer_ros.h>

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <beluga/mixin.hpp>
#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/sensor/likelihood_field_model.hpp>
#include <beluga/random/multivariate_normal_distribution.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <range/v3/algorithm/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "beluga_amcl/amcl_node_utils.hpp"
#include "beluga_amcl/occupancy_grid.hpp"
#include "beluga_amcl/tf2_sophus.hpp"
#include "beluga_amcl/private/execution_policy.hpp"

namespace beluga_amcl
{

namespace
{

constexpr std::string_view kDifferentialModelName = "differential_drive";
constexpr std::string_view kOmnidirectionalModelName = "omnidirectional_drive";
constexpr std::string_view kStationaryModelName = "stationary";

constexpr std::string_view kNav2DifferentialModelName = "nav2_amcl::DifferentialMotionModel";
constexpr std::string_view kNav2OmnidirectionalModelName = "nav2_amcl::OmniMotionModel";

constexpr std::string_view kLikelihoodFieldModelName = "likelihood_field";

}  // namespace

AmclNode::AmclNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode{"amcl", "", options}
{
  RCLCPP_INFO(get_logger(), "Creating");

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "The name of the coordinate frame published by the localization system.";
    declare_parameter("global_frame_id", rclcpp::ParameterValue("map"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "The name of the coordinate frame to use for odometry.";
    declare_parameter("odom_frame_id", rclcpp::ParameterValue("odom"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "The name of the coordinate frame to use for the robot base.";
    declare_parameter("base_frame_id", rclcpp::ParameterValue("base_footprint"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Topic to subscribe to in order to receive the map to localize on.";
    declare_parameter("map_topic", rclcpp::ParameterValue("map"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Topic to subscribe to in order to receive the initial pose of the robot.";
    declare_parameter("initial_pose_topic", rclcpp::ParameterValue("initial_pose"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Topic to subscribe to in order to receive the laser scan for localization.";
    declare_parameter("scan_topic", rclcpp::ParameterValue("scan"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Minimum allowed number of particles.";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    declare_parameter("min_particles", rclcpp::ParameterValue(500), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Maximum allowed number of particles.";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    declare_parameter("max_particles", rclcpp::ParameterValue(2000), descriptor);
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
    declare_parameter("recovery_alpha_slow", rclcpp::ParameterValue(0.001), descriptor);
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
    declare_parameter("recovery_alpha_fast", rclcpp::ParameterValue(0.1), descriptor);
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
    declare_parameter("pf_err", rclcpp::ParameterValue(0.05), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Upper standard normal quantile for P, where P is the probability "
      "that the error in the estimated distribution will be less than pf_err "
      "in KLD resampling.";
    declare_parameter("pf_z", rclcpp::ParameterValue(3.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Resolution in meters for the X axis used to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("spatial_resolution_x", rclcpp::ParameterValue(0.1), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Resolution in meters for the Y axis used to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("spatial_resolution_y", rclcpp::ParameterValue(0.1), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Resolution in radians for the theta axis to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter(
      "spatial_resolution_theta",
      rclcpp::ParameterValue(Sophus::Constants<double>::pi() / 4), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Number of filter updates required before resampling. ";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 1;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    declare_parameter("resample_interval", rclcpp::ParameterValue(1), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "When set to true, will reduce the resampling rate when not needed and help "
      "avoid particle deprivation. The resampling will only happen if the effective "
      "number of particles (N_eff = 1/(sum(k_i^2))) is lower than half the current "
      "number of particles.";
    descriptor.read_only = true;
    declare_parameter("selective_resampling", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Set this to false to prevent amcl from publishing the transform "
      "between the global frame and the odometry frame.";
    declare_parameter("tf_broadcast", rclcpp::ParameterValue(true), descriptor);
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
    declare_parameter("transform_tolerance", rclcpp::ParameterValue(1.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Which motion model to use [differential_drive, omnidirectional_drive, stationary].";
    declare_parameter(
      "robot_model_type",
      rclcpp::ParameterValue(std::string(kDifferentialModelName)), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Rotation noise from rotation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("alpha1", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Rotation noise from translation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("alpha2", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Translation noise from translation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("alpha3", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Translation noise from rotation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("alpha4", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Strafe noise from translation for the omnidirectional drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("alpha5", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Rotational movement required before performing a filter update.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 2 * M_PI;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("update_min_a", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Translational movement required before performing a filter update.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("update_min_d", rclcpp::ParameterValue(0.25), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Which observartion model to use [beam, likelihood_field].";
    declare_parameter(
      "laser_model_type",
      rclcpp::ParameterValue(std::string(kLikelihoodFieldModelName)), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Maximum distance to do obstacle inflation on map, used in likelihood field model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("laser_likelihood_max_dist", rclcpp::ParameterValue(2.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Maximum scan range to be considered.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("laser_max_range", rclcpp::ParameterValue(100.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Minimum scan range to be considered.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("laser_min_range", rclcpp::ParameterValue(0.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "How many evenly-spaced beams in each scan will be used when updating the filter.";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 2;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    declare_parameter("max_beams", rclcpp::ParameterValue(60), descriptor);
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
    descriptor.description = "Standard deviation of the hit distribution.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("sigma_hit", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Set the initial pose from the initial_pose parameters.";
    descriptor.read_only = true;
    declare_parameter("set_initial_pose", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose x axis coordinate.";
    descriptor.read_only = true;
    declare_parameter("initial_pose.x", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose y axis coordinate.";
    descriptor.read_only = true;
    declare_parameter("initial_pose.y", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose z axis coordinate.";
    descriptor.read_only = true;
    declare_parameter("initial_pose.z", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yaw rotation.";
    descriptor.read_only = true;
    declare_parameter("initial_pose.yaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose x axis covariance.";
    descriptor.read_only = true;
    declare_parameter("initial_pose.covariance_x", 0.5 * 0.5, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose y axis covariance.";
    descriptor.read_only = true;
    declare_parameter("initial_pose.covariance_y", 0.5 * 0.5, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yaw covariance.";
    descriptor.read_only = true;
    declare_parameter(
      "initial_pose.covariance_yaw",
      Sophus::Constants<double>::pi() / 12. * Sophus::Constants<double>::pi() / 12.,
      descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose xy covariance.";
    descriptor.read_only = true;
    declare_parameter("initial_pose.covariance_xy", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose xyaw covariance.";
    descriptor.read_only = true;
    declare_parameter("initial_pose.covariance_xyaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yyaw covariance.";
    descriptor.read_only = true;
    declare_parameter("initial_pose.covariance_yyaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.read_only = true;
    descriptor.description =
      "Execution policy used to process particles [seq, par].";
    auto execution_policy_string = declare_parameter(
      "execution_policy", "par", descriptor);
    try {
      execution_policy_ = beluga_amcl::execution::policy_from_string(execution_policy_string);
    } catch (const std::invalid_argument &) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "execution_policy param should be [seq, par], got: " <<
          execution_policy_string << "\nUsing the default parallel policy.");
      execution_policy_ = std::execution::par;
    }
  }
}

AmclNode::~AmclNode()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  // In case this lifecycle node wasn't properly shut down, do it here
  if (get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    on_deactivate(get_current_state());
    on_cleanup(get_current_state());
  }
}

AmclNode::CallbackReturn AmclNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(200ms, std::bind(&AmclNode::timer_callback, this));

  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud",
    rclcpp::SensorDataQoS());

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose",
    rclcpp::SystemDefaultsQoS());

  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  particle_cloud_pub_->on_activate();
  pose_pub_->on_activate();

  RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", get_name());

  bond_ = std::make_unique<bond::Bond>("bond", get_name(), shared_from_this());
  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    get_parameter("map_topic").as_string(),
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&AmclNode::map_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribed to map_topic: %s", map_sub_->get_topic_name());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    get_parameter("initial_pose_topic").as_string(),
    rclcpp::SystemDefaultsQoS(),
    std::bind(&AmclNode::initial_pose_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Subscribed to initial_pose_topic: %s",
    initial_pose_sub_->get_topic_name());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setCreateTimerInterface(
    std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface()));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
    *tf_buffer_,
    this,
    false);  // avoid using dedicated tf thread

  laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
      rclcpp_lifecycle::LifecycleNode>>(
    shared_from_this(), get_parameter("scan_topic").as_string(), rmw_qos_profile_sensor_data);

  // Message filter that caches laser scan readings until it is possible to transform
  // from laser frame to odom frame and update the particle filter.
  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, get_parameter("odom_frame_id").as_string(), 50,
    get_node_logging_interface(),
    get_node_clock_interface(),
    tf2::durationFromSec(1.0));

  using LaserCallback = std::function<void (sensor_msgs::msg::LaserScan::ConstSharedPtr)>;
  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::visit(
      [this](const auto & policy) -> LaserCallback
      {
        return [this, &policy](sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
          this->laser_callback(policy, std::move(laser_scan));
        };
      }, execution_policy_));
  RCLCPP_INFO(get_logger(), "Subscribed to scan_topic: %s", laser_scan_sub_->getTopic().c_str());

  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_deactivate(const rclcpp_lifecycle::State &)
{
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
  bond_.reset();
  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  particle_cloud_pub_.reset();
  pose_pub_.reset();
  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}

void AmclNode::map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  RCLCPP_INFO(get_logger(), "A new map was received");

  auto sampler_params = beluga::AdaptiveSamplerParam{};
  sampler_params.alpha_slow = get_parameter("recovery_alpha_slow").as_double();
  sampler_params.alpha_fast = get_parameter("recovery_alpha_fast").as_double();

  auto limiter_params = beluga::KldLimiterParam<Sophus::SE2d>{};
  limiter_params.min_samples = static_cast<std::size_t>(get_parameter("min_particles").as_int());
  limiter_params.max_samples = static_cast<std::size_t>(get_parameter("max_particles").as_int());
  limiter_params.spatial_hasher = beluga::spatial_hash<Sophus::SE2d>{
    get_parameter("spatial_resolution_x").as_double(),
    get_parameter("spatial_resolution_y").as_double(),
    get_parameter("spatial_resolution_theta").as_double(),
  };
  limiter_params.kld_epsilon = get_parameter("pf_err").as_double();
  limiter_params.kld_z = get_parameter("pf_z").as_double();

  auto resample_on_motion_params = beluga::ResampleOnMotionPolicyParam{};
  resample_on_motion_params.update_min_d = get_parameter("update_min_d").as_double();
  resample_on_motion_params.update_min_a = get_parameter("update_min_a").as_double();

  auto resample_interval_params = beluga::ResampleIntervalPolicyParam{};
  resample_interval_params.resample_interval_count =
    static_cast<std::size_t>(get_parameter("resample_interval").as_int());

  auto selective_resampling_params = beluga::SelectiveResamplingPolicyParam{};
  selective_resampling_params.enabled = get_parameter("selective_resampling").as_bool();

  using Stationary = beluga::mixin::descriptor<beluga::StationaryModel>;
  using DifferentialDrive = beluga::mixin::descriptor<
    beluga::DifferentialDriveModel,
    beluga::DifferentialDriveModelParam>;
  using OmnidirectionalDrive = beluga::mixin::descriptor<
    beluga::OmnidirectionalDriveModel,
    beluga::OmnidirectionalDriveModelParam>;

  using MotionDescriptor = std::variant<Stationary, DifferentialDrive, OmnidirectionalDrive>;
  auto get_motion_descriptor = [this](std::string_view name) -> MotionDescriptor {
      if (name == kDifferentialModelName || name == kNav2DifferentialModelName) {
        auto params = beluga::DifferentialDriveModelParam{};
        params.rotation_noise_from_rotation = get_parameter("alpha1").as_double();
        params.rotation_noise_from_translation = get_parameter("alpha2").as_double();
        params.translation_noise_from_translation = get_parameter("alpha3").as_double();
        params.translation_noise_from_rotation = get_parameter("alpha4").as_double();
        return DifferentialDrive{params};
      } else if (name == kOmnidirectionalModelName || name == kNav2OmnidirectionalModelName) {
        auto params = beluga::OmnidirectionalDriveModelParam{};
        params.rotation_noise_from_rotation = get_parameter("alpha1").as_double();
        params.rotation_noise_from_translation = get_parameter("alpha2").as_double();
        params.translation_noise_from_translation = get_parameter("alpha3").as_double();
        params.translation_noise_from_rotation = get_parameter("alpha4").as_double();
        params.strafe_noise_from_translation = get_parameter("alpha5").as_double();
        return OmnidirectionalDrive{params};
      } else if (name == kStationaryModelName) {
        return Stationary{};
      }
      throw std::invalid_argument(std::string("Invalid motion model: ") + std::string(name));
    };

  using LikelihoodField = beluga::mixin::descriptor<
    ciabatta::curry<beluga::LikelihoodFieldModel, OccupancyGrid>::mixin,
    beluga::LikelihoodFieldModelParam>;

  using SensorDescriptor = std::variant<LikelihoodField>;
  auto get_sensor_descriptor = [this](std::string_view name) -> SensorDescriptor {
      if (name == kLikelihoodFieldModelName) {
        auto params = beluga::LikelihoodFieldModelParam{};
        params.max_obstacle_distance = get_parameter("laser_likelihood_max_dist").as_double();
        params.max_laser_distance = get_parameter("laser_max_range").as_double();
        params.z_hit = get_parameter("z_hit").as_double();
        params.z_random = get_parameter("z_rand").as_double();
        params.sigma_hit = get_parameter("sigma_hit").as_double();
        return LikelihoodField{params};
      }
      throw std::invalid_argument(std::string("Invalid sensor model: ") + std::string(name));
    };

  // Only when we get the first map we should use the parameters, not later.
  // TODO(ivanpauno): Intialize later maps from last known pose.
  const bool initialize_from_params = !particle_filter_ &&
    this->get_parameter("set_initial_pose").as_bool();

  try {
    using beluga::mixin::make_mixin;
    using beluga::LaserLocalizationInterface2d;
    using beluga::AdaptiveMonteCarloLocalization2d;
    particle_filter_ = make_mixin<LaserLocalizationInterface2d, AdaptiveMonteCarloLocalization2d>(
      sampler_params,
      limiter_params,
      resample_on_motion_params,
      resample_interval_params,
      selective_resampling_params,
      get_motion_descriptor(get_parameter("robot_model_type").as_string()),
      get_sensor_descriptor(get_parameter("laser_model_type").as_string()),
      OccupancyGrid{map}
    );
  } catch (const std::invalid_argument & error) {
    RCLCPP_ERROR(this->get_logger(), "Coudn't instantiate the particle filter: %s", error.what());
    return;
  }

  if (initialize_from_params) {
    Eigen::Vector3d mean;
    Eigen::Matrix3d covariance;

    mean.x() = this->get_parameter("initial_pose.x").as_double();
    mean.y() = this->get_parameter("initial_pose.y").as_double();
    mean.z() = this->get_parameter("initial_pose.yaw").as_double();
    covariance.coeffRef(0, 0) = this->get_parameter("initial_pose.covariance_x").as_double();
    covariance.coeffRef(1, 1) = this->get_parameter("initial_pose.covariance_y").as_double();
    covariance.coeffRef(2, 2) = this->get_parameter("initial_pose.covariance_yaw").as_double();
    covariance.coeffRef(0, 1) = this->get_parameter("initial_pose.covariance_xy").as_double();
    covariance.coeffRef(1, 0) = covariance.coeffRef(0, 1);
    covariance.coeffRef(0, 2) = this->get_parameter("initial_pose.covariance_xyaw").as_double();
    covariance.coeffRef(2, 0) = covariance.coeffRef(0, 2);
    covariance.coeffRef(1, 2) = this->get_parameter("initial_pose.covariance_yyaw").as_double();
    covariance.coeffRef(2, 1) = covariance.coeffRef(1, 2);
    this->reinitialize_with_pose(mean, covariance);

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Particle filter initialized with initial pose x=" <<
        mean.x() << ", y=" << mean.y() << ", yaw=" << mean.z());
  }

  RCLCPP_INFO(
    get_logger(), "Particle filter initialized with %ld particles",
    particle_filter_->particle_count());
}

void AmclNode::timer_callback()
{
  if (!particle_filter_) {
    return;
  }

  if (particle_cloud_pub_->get_subscription_count() == 0) {
    return;
  }

  auto message = nav2_msgs::msg::ParticleCloud{};
  message.header.stamp = now();
  message.header.frame_id = get_parameter("global_frame_id").as_string();
  message.particles.resize(particle_filter_->particle_count());
  ranges::transform(
    ranges::views::zip(particle_filter_->states_view(), particle_filter_->weights_view()),
    std::begin(message.particles), [](const auto & particle) {
      const auto & [state, weight] = particle;
      auto message = nav2_msgs::msg::Particle{};
      tf2::toMsg(state, message.pose);
      message.weight = weight;
      return message;
    });
  particle_cloud_pub_->publish(message);
}

template<typename ExecutionPolicy>
void AmclNode::laser_callback(
  ExecutionPolicy && exec_policy,
  sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
  if (!particle_filter_) {
    return;
  }

  auto odom_to_base_transform = Sophus::SE2d{};
  try {
    // Use the lookupTransform overload with no timeout since we're not using a dedicated
    // tf thread. The message filter we are using avoids the need for it.
    tf2::convert(
      tf_buffer_->lookupTransform(
        get_parameter("odom_frame_id").as_string(),
        get_parameter("base_frame_id").as_string(),
        tf2_ros::fromMsg(laser_scan->header.stamp)).transform,
      odom_to_base_transform);
  } catch (const tf2::TransformException & error) {
    RCLCPP_ERROR(get_logger(), "Could not transform from odom to base: %s", error.what());
    return;
  }

  auto base_to_laser_transform = Sophus::SE3d{};
  try {
    tf2::convert(
      tf_buffer_->lookupTransform(
        get_parameter("base_frame_id").as_string(),
        laser_scan->header.frame_id,
        tf2_ros::fromMsg(laser_scan->header.stamp)).transform,
      base_to_laser_transform);
  } catch (const tf2::TransformException & error) {
    RCLCPP_ERROR(get_logger(), "Could not transform from base to laser: %s", error.what());
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
        static_cast<std::size_t>(get_parameter("max_beams").as_int()),
        static_cast<float>(get_parameter("laser_min_range").as_double()),
        static_cast<float>(get_parameter("laser_max_range").as_double())));
    particle_filter_->importance_sample(exec_policy);
    const auto time3 = std::chrono::high_resolution_clock::now();
    particle_filter_->resample();
    const auto time4 = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Particle filter update stats: %ld particles %ld points - %.3fms %.3fms %.3fms",
      particle_filter_->particle_count(),
      laser_scan->ranges.size(),
      std::chrono::duration<double, std::milli>(time2 - time1).count(),
      std::chrono::duration<double, std::milli>(time3 - time2).count(),
      std::chrono::duration<double, std::milli>(time4 - time3).count());
  }

  const auto [pose, covariance] = particle_filter_->estimate();

  {
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped{};
    message.header.stamp = laser_scan->header.stamp;
    message.header.frame_id = get_parameter("global_frame_id").as_string();
    tf2::toMsg(pose, message.pose.pose);
    message.pose.covariance = tf2::covarianceEigenToRowMajor(covariance);
    pose_pub_->publish(message);
  }

  if (get_parameter("tf_broadcast").as_bool()) {
    auto message = geometry_msgs::msg::TransformStamped{};
    // Sending a transform that is valid into the future so that odom can be used.
    message.header.stamp = now() +
      tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
    message.header.frame_id = get_parameter("global_frame_id").as_string();
    message.child_frame_id = get_parameter("odom_frame_id").as_string();
    message.transform = tf2::toMsg(odom_to_base_transform * pose.inverse());
    tf_broadcaster_->sendTransform(message);
  }
}

void AmclNode::initial_pose_callback(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
  if (!particle_filter_) {
    return;
  }

  const auto global_frame_id = get_parameter("global_frame_id").as_string();
  if (message->header.frame_id != global_frame_id) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring initial pose in frame \"%s\"; it must be in the global frame \"%s\"",
      message->header.frame_id.c_str(),
      global_frame_id.c_str());
  }

  auto pose = Sophus::SE2d{};
  tf2::convert(message->pose.pose, pose);

  auto covariance = Eigen::Matrix3d{};
  tf2::covarianceRowMajorToEigen(message->pose.covariance, covariance);

  const auto mean =
    Eigen::Vector3d{pose.translation().x(), pose.translation().y(), pose.so2().log()};

  {
    const auto eigen_format = utils::make_eigen_comma_format();
    RCLCPP_INFO(get_logger(), "Initial pose received");
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Mean value: " << mean.format(eigen_format) <<
        " - Covariance coefficients: " << covariance.format(eigen_format));
  }
  this->reinitialize_with_pose(mean, covariance);
}

void AmclNode::reinitialize_with_pose(
  const Eigen::Vector3d & mean, const Eigen::Matrix3d & covariance)
{
  try {
    particle_filter_->initialize_states(
      ranges::views::generate(
        [distribution = beluga::MultivariateNormalDistribution{mean, covariance}]() mutable {
          static auto generator = std::mt19937{std::random_device()()};
          const auto sample = distribution(generator);
          return Sophus::SE2d{Sophus::SO2d{sample.z()}, Eigen::Vector2d{sample.x(), sample.y()}};
        }));
  } catch (const std::runtime_error & error) {
    RCLCPP_ERROR(get_logger(), "Could not generate particles: %s", error.what());
  }
}

}  // namespace beluga_amcl

RCLCPP_COMPONENTS_REGISTER_NODE(beluga_amcl::AmclNode)

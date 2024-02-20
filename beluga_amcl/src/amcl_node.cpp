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

constexpr std::string_view kDifferentialModelName = "differential_drive";
constexpr std::string_view kOmnidirectionalModelName = "omnidirectional_drive";
constexpr std::string_view kStationaryModelName = "stationary";

constexpr std::string_view kLikelihoodFieldModelName = "likelihood_field";
constexpr std::string_view kBeamSensorModelName = "beam";

constexpr std::string_view kNav2DifferentialModelName = "nav2_amcl::DifferentialMotionModel";
constexpr std::string_view kNav2OmnidirectionalModelName = "nav2_amcl::OmniMotionModel";

}  // namespace

AmclNode::AmclNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode{"amcl", "", options} {
  RCLCPP_INFO(get_logger(), "Creating");

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "The name of the coordinate frame published by the localization system.";
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
    descriptor.description = "Topic to subscribe to in order to receive the map to localize on.";
    declare_parameter("map_topic", rclcpp::ParameterValue("map"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Topic to subscribe to in order to receive the initial pose of the robot.";
    declare_parameter("initial_pose_topic", rclcpp::ParameterValue("initialpose"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Topic to subscribe to in order to receive the laser scan for localization.";
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
    declare_parameter("recovery_alpha_slow", rclcpp::ParameterValue(0.0), descriptor);
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
    declare_parameter("recovery_alpha_fast", rclcpp::ParameterValue(0.0), descriptor);
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
    declare_parameter("pf_z", rclcpp::ParameterValue(0.99), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Resolution in meters for the X axis used to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("spatial_resolution_x", rclcpp::ParameterValue(0.5), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Resolution in meters for the Y axis used to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("spatial_resolution_y", rclcpp::ParameterValue(0.5), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Resolution in radians for the theta axis to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 2 * Sophus::Constants<double>::pi();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter(
        "spatial_resolution_theta", rclcpp::ParameterValue(10 * Sophus::Constants<double>::pi() / 180), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Number of filter updates required before resampling. ";
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
    descriptor.description = "Which motion model to use [differential_drive, omnidirectional_drive, stationary].";
    declare_parameter("robot_model_type", rclcpp::ParameterValue(std::string(kDifferentialModelName)), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Rotation noise from rotation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("alpha1", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Rotation noise from translation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("alpha2", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Translation noise from translation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("alpha3", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Translation noise from rotation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("alpha4", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Strafe noise from translation for the omnidirectional drive model.";
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
    descriptor.floating_point_range[0].to_value = 2 * Sophus::Constants<double>::pi();
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
    descriptor.description = "How many evenly-spaced beams in each scan will be used when updating the filter.";
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
    descriptor.description = "Set the initial pose from the initial_pose parameters.";
    declare_parameter("set_initial_pose", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose x axis coordinate.";
    declare_parameter("initial_pose.x", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose y axis coordinate.";
    declare_parameter("initial_pose.y", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yaw rotation.";
    declare_parameter("initial_pose.yaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose x axis covariance.";
    declare_parameter("initial_pose.covariance_x", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose y axis covariance.";
    declare_parameter("initial_pose.covariance_y", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yaw covariance.";
    declare_parameter("initial_pose.covariance_yaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose xy covariance.";
    declare_parameter("initial_pose.covariance_xy", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose xyaw covariance.";
    declare_parameter("initial_pose.covariance_xyaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yyaw covariance.";
    declare_parameter("initial_pose.covariance_yyaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Execution policy used to process particles [seq, par].";
    descriptor.read_only = true;
    declare_parameter("execution_policy", "seq", descriptor);
  }
}

AmclNode::~AmclNode() {
  RCLCPP_INFO(get_logger(), "Destroying");
  // In case this lifecycle node wasn't properly shut down, do it here
  on_shutdown(get_current_state());
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
  impl_.reset();
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

auto AmclNode::get_initial_estimate() -> std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> {
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

auto AmclNode::get_motion_model(std::string_view name) -> beluga_ros::Amcl::motion_model_variant {
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

auto AmclNode::get_sensor_model(std::string_view name, nav_msgs::msg::OccupancyGrid::SharedPtr map)
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

auto AmclNode::get_amcl_impl(nav_msgs::msg::OccupancyGrid::SharedPtr map) -> std::unique_ptr<beluga_ros::Amcl> {
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

  try {
    return std::make_unique<beluga_ros::Amcl>(
        beluga_ros::OccupancyGrid{map},                                        //
        get_motion_model(get_parameter("robot_model_type").as_string()),       //
        get_sensor_model(get_parameter("laser_model_type").as_string(), map),  //
        params,                                                                //
        get_execution_policy(get_parameter("execution_policy").as_string()));
  } catch (const std::invalid_argument& error) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particle filter: %s", error.what());
  }

  return nullptr;
}

void AmclNode::map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  RCLCPP_INFO(get_logger(), "A new map was received");

  if (impl_ && get_parameter("first_map_only").as_bool()) {
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
                                         (!impl_ && !last_known_estimate_.has_value());

  if (!impl_) {
    impl_ = get_amcl_impl(map);
  } else {
    impl_->update_map(beluga_ros::OccupancyGrid{std::move(map)});
  }

  if (!impl_) {
    return;  // Initialization failed and the error was already logged.
  }

  const auto initial_estimate = get_initial_estimate();
  if (should_reset_initial_pose && initial_estimate.has_value()) {
    last_known_estimate_ = initial_estimate;
  }

  if (last_known_estimate_.has_value()) {
    initialize_from_estimate(last_known_estimate_.value());
  } else {
    initialize_from_map();
  }
}

void AmclNode::timer_callback() {
  if (!impl_) {
    return;
  }

  if (particle_cloud_pub_->get_subscription_count() == 0) {
    return;
  }

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
  // To handle both cases, we group repeated candidates and compute the accumulated weight

  struct RepresentativeData {
    Sophus::SE2d state;
    double weight{0.};
  };

  struct RepresentativeBinHash {
    std::size_t operator()(const Sophus::SE2d& s) const noexcept {
      std::size_t h1 = std::hash<double>{}(s.translation().x());
      std::size_t h2 = std::hash<double>{}(s.translation().y());
      std::size_t h3 = std::hash<double>{}(s.so2().log());
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  struct RepresentativeBinEqual {
    bool operator()(const Sophus::SE2d& lhs, const Sophus::SE2d& rhs) const noexcept {
      // good enough, since copies of the same candidate are expected to be identical copies
      return lhs.translation().x() == rhs.translation().x() &&  //
             lhs.translation().y() == rhs.translation().y() &&  //
             lhs.so2().log() == rhs.so2().log();
    }
  };

  std::unordered_map<Sophus::SE2d, RepresentativeData, RepresentativeBinHash, RepresentativeBinEqual>
      representatives_map;
  representatives_map.reserve(impl_->particles().size());
  double max_weight = 1e-5;  // never risk dividing by zero

  for (const auto& [state, weight] : impl_->particles()) {
    auto& representative = representatives_map[state];  // if the element does not exist, create it
    representative.state = state;
    representative.weight += weight;
    if (representative.weight > max_weight) {
      max_weight = representative.weight;
    }
  }

  auto message = nav2_msgs::msg::ParticleCloud{};
  message.header.stamp = now();
  message.header.frame_id = get_parameter("global_frame_id").as_string();
  message.particles.reserve(impl_->particles().size());

  for (const auto& [key, representative] : representatives_map) {
    auto& particle = message.particles.emplace_back();
    tf2::toMsg(representative.state, particle.pose);
    particle.weight = representative.weight / max_weight;
  }

  particle_cloud_pub_->publish(message);
}

void AmclNode::laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
  if (!impl_) {
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
  auto new_estimate = impl_->update(
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
    last_known_estimate_ = new_estimate;

    RCLCPP_INFO(
        get_logger(), "Particle filter update iteration stats: %ld particles %ld points - %.3fms",
        impl_->particles().size(), laser_scan->ranges.size(),
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
    auto odom_transform_in_map = base_pose_in_map * base_pose_in_odom.inverse();
    auto message = geometry_msgs::msg::TransformStamped{};
    // Sending a transform that is valid into the future so that odom can be used.
    const auto expiration_stamp = tf2_ros::fromMsg(laser_scan->header.stamp) +
                                  tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
    message.header.stamp = tf2_ros::toMsg(expiration_stamp);
    message.header.frame_id = get_parameter("global_frame_id").as_string();
    message.child_frame_id = get_parameter("odom_frame_id").as_string();
    message.transform = tf2::toMsg(odom_transform_in_map);
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
  initialize_from_estimate(last_known_estimate_.value());
}

void AmclNode::global_localization_callback(
    [[maybe_unused]] std::shared_ptr<rmw_request_id_t> request_header,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Request> req,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> res) {
  initialize_from_map();
  enable_tf_broadcast_ = true;
}

void AmclNode::nomotion_update_callback(
    [[maybe_unused]] std::shared_ptr<rmw_request_id_t> request_header,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Request> req,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> res) {
  if (!impl_) {
    RCLCPP_WARN(get_logger(), "Ignoring no-motion update request because the particle filter has not been initialized");
    return;
  }

  impl_->force_update();
  RCLCPP_INFO(get_logger(), "No-motion update requested");
}

bool AmclNode::initialize_from_estimate(const std::pair<Sophus::SE2d, Eigen::Matrix3d>& estimate) {
  RCLCPP_INFO(get_logger(), "Initializing particles from estimated pose and covariance");

  if (!impl_) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particles: The particle filter has not been initialized");
    return false;
  }

  const auto& [pose, covariance] = estimate;

  try {
    impl_->initialize(pose, covariance);
  } catch (const std::runtime_error& error) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particles: %s", error.what());
    return false;
  }

  enable_tf_broadcast_ = true;

  RCLCPP_INFO(
      get_logger(), "Particle filter initialized with %ld particles about initial pose x=%g, y=%g, yaw=%g",
      impl_->particles().size(), pose.translation().x(), pose.translation().y(), pose.so2().log());

  return true;
}

bool AmclNode::initialize_from_map() {
  RCLCPP_INFO(get_logger(), "Initializing particles from map");

  if (!impl_) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particles: The particle filter has not been initialized");
    return false;
  }

  impl_->initialize_from_map();

  // NOTE: We do not set `enable_tf_broadcast_ = true` here to match the original implementation.

  RCLCPP_INFO(
      get_logger(), "Particle filter initialized with %ld particles distributed across the map",
      impl_->particles().size());

  return true;
}

}  // namespace beluga_amcl

RCLCPP_COMPONENTS_REGISTER_NODE(beluga_amcl::AmclNode)

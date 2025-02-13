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

#include "beluga_amcl/ros2_common.hpp"
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/publisher.hpp>

namespace beluga_amcl {

BaseAMCLNode::BaseAMCLNode(
    const std::string& node_name,
    const std::string& node_namespace,
    const rclcpp::NodeOptions& node_options)
    : rclcpp_lifecycle::LifecycleNode(node_name, node_namespace, node_options) {
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "The name of the coordinate frame published by the localization system.";
    this->declare_parameter("global_frame_id", rclcpp::ParameterValue("map"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "The name of the coordinate frame to use for odometry.";
    this->declare_parameter("odom_frame_id", rclcpp::ParameterValue("odom"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "The name of the coordinate frame to use for the robot base.";
    this->declare_parameter("base_frame_id", rclcpp::ParameterValue("base_footprint"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Path to load the map from an hdf5 file.";
    this->declare_parameter("map_path", rclcpp::ParameterValue("map_path"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Topic to subscribe to in order to receive the initial pose of the robot.";
    this->declare_parameter("initial_pose_topic", rclcpp::ParameterValue("initialpose"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Topic to subscribe to in order to receive the laser scan for localization.";
    this->declare_parameter("scan_topic", rclcpp::ParameterValue("scan"), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Minimum allowed number of particles.";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    this->declare_parameter("min_particles", rclcpp::ParameterValue(500), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Maximum allowed number of particles.";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 0;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    this->declare_parameter("max_particles", rclcpp::ParameterValue(2000), descriptor);
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
    this->declare_parameter("recovery_alpha_slow", rclcpp::ParameterValue(0.0), descriptor);
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
    this->declare_parameter("recovery_alpha_fast", rclcpp::ParameterValue(0.0), descriptor);
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
    this->declare_parameter("pf_err", rclcpp::ParameterValue(0.05), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Upper standard normal quantile for P, where P is the probability "
        "that the error in the estimated distribution will be less than pf_err "
        "in KLD resampling.";
    this->declare_parameter("pf_z", rclcpp::ParameterValue(0.99), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Resolution in meters for the X axis used to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("spatial_resolution_x", rclcpp::ParameterValue(0.5), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Resolution in meters for the Y axis used to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("spatial_resolution_y", rclcpp::ParameterValue(0.5), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Resolution in radians for the theta axis to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 2 * Sophus::Constants<double>::pi();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter(
        "spatial_resolution_theta", rclcpp::ParameterValue(10 * Sophus::Constants<double>::pi() / 180), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Number of filter updates required before resampling. ";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 1;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    this->declare_parameter("resample_interval", rclcpp::ParameterValue(1), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "When set to true, will reduce the resampling rate when not needed and help "
        "avoid particle deprivation. The resampling will only happen if the effective "
        "number of particles (N_eff = 1/(sum(k_i^2))) is lower than half the current "
        "number of particles.";
    descriptor.read_only = true;
    this->declare_parameter("selective_resampling", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
        "Set this to false to prevent amcl from publishing the transform "
        "between the global frame and the odometry frame.";
    this->declare_parameter("tf_broadcast", rclcpp::ParameterValue(true), descriptor);
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
    this->declare_parameter("transform_tolerance", rclcpp::ParameterValue(1.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Which motion model to use [differential_drive, omnidirectional_drive, stationary].";
    this->declare_parameter(
        "robot_model_type", rclcpp::ParameterValue(std::string(kDifferentialModelName)), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Rotation noise from rotation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("alpha1", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Rotation noise from translation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("alpha2", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Translation noise from translation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("alpha3", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Translation noise from rotation for the differential drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("alpha4", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Strafe noise from translation for the omnidirectional drive model.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("alpha5", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Rotational movement required before performing a filter update.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 2 * Sophus::Constants<double>::pi();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("update_min_a", rclcpp::ParameterValue(0.2), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Translational movement required before performing a filter update.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("update_min_d", rclcpp::ParameterValue(0.25), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Maximum scan range to be considered.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("laser_max_range", rclcpp::ParameterValue(100.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Minimum scan range to be considered.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    this->declare_parameter("laser_min_range", rclcpp::ParameterValue(0.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "How many evenly-spaced beams in each scan will be used when updating the filter.";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 2;
    descriptor.integer_range[0].to_value = std::numeric_limits<int>::max();
    descriptor.integer_range[0].step = 1;
    this->declare_parameter("max_beams", rclcpp::ParameterValue(60), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Set the initial pose from the initial_pose parameters.";
    this->declare_parameter("set_initial_pose", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose x axis coordinate.";
    this->declare_parameter("initial_pose.x", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose y axis coordinate.";
    this->declare_parameter("initial_pose.y", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yaw rotation.";
    this->declare_parameter("initial_pose.yaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose x axis covariance.";
    this->declare_parameter("initial_pose.covariance_x", 1e-6, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose y axis covariance.";
    this->declare_parameter("initial_pose.covariance_y", 1e-6, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yaw covariance.";
    this->declare_parameter("initial_pose.covariance_yaw", 1e-6, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose xy covariance.";
    this->declare_parameter("initial_pose.covariance_xy", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose xyaw covariance.";
    this->declare_parameter("initial_pose.covariance_xyaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Initial pose yyaw covariance.";
    this->declare_parameter("initial_pose.covariance_yyaw", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Execution policy used to process particles [seq, par].";
    this->declare_parameter("execution_policy", "seq", descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Whether the this->should configure and activate itself or not.";
    this->declare_parameter("autostart", false, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Delay, in seconds, before autostarting if autostarting.";
    this->declare_parameter("autostart_delay", 0.0, descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Heartbeat timeout value for the bond connection.";
    this->declare_parameter("bond_timeout", 4.0, descriptor);
  }

  common_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  common_subscription_options_ = rclcpp::SubscriptionOptions{};
  common_subscription_options_.callback_group = common_callback_group_;

  if (get_parameter("autostart").as_bool()) {
    auto autostart_delay = std::chrono::duration<double>(get_parameter("autostart_delay").as_double());
    autostart_timer_ = create_wall_timer(autostart_delay, std::bind(&BaseAMCLNode::autostart_callback, this));
  }
}

BaseAMCLNode::~BaseAMCLNode() {
  RCLCPP_INFO(get_logger(), "Destroying");
  // In case this lifecycle node wasn't properly shut down, do it here
  on_shutdown(get_current_state());
}

/// Callback for lifecycle transitions from the UNCONFIGURED state to the INACTIVE state.
BaseAMCLNode::CallbackReturn BaseAMCLNode::on_configure(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Configuring");
  particle_cloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", rclcpp::SensorDataQoS());
  particle_markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("particle_markers", rclcpp::SystemDefaultsQoS());
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", rclcpp::SystemDefaultsQoS());
  do_configure(state);
  return CallbackReturn::SUCCESS;
};

BaseAMCLNode::CallbackReturn BaseAMCLNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Deactivating");
  particle_cloud_pub_->on_deactivate();
  particle_markers_pub_->on_deactivate();
  pose_pub_->on_deactivate();
  initial_pose_sub_.reset();
  tf_listener_.reset();
  tf_broadcaster_.reset();
  tf_buffer_.reset();
  bond_.reset();
  do_deactivate(state);
  return CallbackReturn::SUCCESS;
}

BaseAMCLNode::CallbackReturn BaseAMCLNode::on_shutdown(const rclcpp_lifecycle::State& state) {
  using lifecycle_msgs::msg::State;
  RCLCPP_INFO(get_logger(), "Shutting down");
  if (state.id() == State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(state);
    on_cleanup(state);
  }
  if (state.id() == State::PRIMARY_STATE_INACTIVE) {
    on_cleanup(state);
  }
  do_shutdown(state);
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BaseAMCLNode::on_cleanup(
    const rclcpp_lifecycle::State& state) {
  do_cleanup(state);
  return CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BaseAMCLNode::on_activate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Activating");
  particle_cloud_pub_->on_activate();
  particle_markers_pub_->on_activate();
  pose_pub_->on_activate();

  {
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        get_parameter("initial_pose_topic").as_string(), rclcpp::SystemDefaultsQoS(),
        std::bind(&BaseAMCLNode::initial_pose_callback, this, std::placeholders::_1), common_subscription_options_);
    RCLCPP_INFO(get_logger(), "Subscribed to initial_pose_topic: %s", initial_pose_sub_->get_topic_name());
  }

  {
    using namespace std::chrono_literals;
    // TODO(alon): create a parameter for the timer rate?
    timer_ = create_wall_timer(200ms, std::bind(&BaseAMCLNode::periodic_timer_callback, this), common_callback_group_);
  }

  {
    auto on_bond_formed_callback = [this]() {
      RCLCPP_INFO(get_logger(), "The bond connection to the lifecycle manager is now fully formed");
    };
    auto on_bond_broken_callback = [this]() {
      RCLCPP_ERROR(get_logger(), "The bond connection to the lifecycle manager has been broken");
    };
    bond_ = std::make_unique<bond::Bond>(
        "bond", get_name(), shared_from_this(), on_bond_broken_callback, on_bond_formed_callback);
    bond_->setHeartbeatPeriod(0.10);
    const auto heartbeat_timeout_value = get_parameter("bond_timeout").as_double();
    // we don't want to shorten the default connection timeout, but we want to be able to
    // make it longer because it can fail while building the likelihood map for large maps
    // just like the heartbeat timeout does
    const auto connect_timeout_value =
        std::max(heartbeat_timeout_value, static_cast<double>(bond::msg::Constants::DEFAULT_CONNECT_TIMEOUT));
    bond_->setConnectTimeout(connect_timeout_value);
    bond_->setHeartbeatTimeout(heartbeat_timeout_value);
    bond_->start();
    RCLCPP_INFO(
        get_logger(),
        "The bond (%s) connection to the lifecycle manager has been started (heartbeat timeout: %.2lf seconds)",
        get_name(), heartbeat_timeout_value);
  }

  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface()));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
        *tf_buffer_, this,
        false);  // avoid using dedicated tf thread
  }

  do_activate(state);
  return CallbackReturn::SUCCESS;
}

void BaseAMCLNode::initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
  const auto global_frame_id = get_parameter("global_frame_id").as_string();
  if (message->header.frame_id != global_frame_id) {
    RCLCPP_WARN(
        get_logger(), "Ignoring initial pose in frame \"%s\"; it must be in the global frame \"%s\"",
        message->header.frame_id.c_str(), global_frame_id.c_str());
    return;
  }
  do_initial_pose_callback(message);
}

void BaseAMCLNode::periodic_timer_callback() {
  do_periodic_timer_callback();
};

void BaseAMCLNode::autostart_callback() {
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
  do_autostart_callback();
  autostart_timer_->cancel();
}

auto BaseAMCLNode::get_execution_policy() const -> ExecutionPolicyVariant {
  const auto name = get_parameter("execution_policy").as_string();
  if (name == "seq") {
    return std::execution::seq;
  }
  if (name == "par") {
    return std::execution::par;
  }
  throw std::invalid_argument("Execution policy must be seq or par.");
}
}  // namespace beluga_amcl

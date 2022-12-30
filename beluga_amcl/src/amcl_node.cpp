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

#include <beluga_amcl/amcl_node.hpp>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/create_timer_ros.h>

#include <chrono>
#include <limits>
#include <memory>

#include <beluga_amcl/convert.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <range/v3/range/conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace beluga_amcl
{

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
      "Spatial resolution used to divide the space in buckets for KLD resampling.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("spatial_resolution", rclcpp::ParameterValue(0.1), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description =
      "Number of filter updates required before resampling. "
      "Values other than 1 are not supported in this implementation.";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 1;
    descriptor.integer_range[0].to_value = 1;
    descriptor.integer_range[0].step = 1;
    declare_parameter("resample_interval", rclcpp::ParameterValue(1), descriptor);
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
    descriptor.description = "Mixin weight for the probability of hitting an obstacle.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("z_hit", rclcpp::ParameterValue(0.5), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Mixin weight for the probability of getting random measurements.";
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
  timer_ = create_wall_timer(500ms, std::bind(&AmclNode::timer_callback, this));

  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud",
    rclcpp::SensorDataQoS());

  likelihood_field_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "likelihood_field",
    rclcpp::SystemDefaultsQoS());

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose",
    rclcpp::SystemDefaultsQoS());

  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  particle_cloud_pub_->on_activate();
  likelihood_field_pub_->on_activate();
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

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setCreateTimerInterface(
    std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface()));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
      rclcpp_lifecycle::LifecycleNode>>(
    shared_from_this(), get_parameter("scan_topic").as_string(), rmw_qos_profile_sensor_data);

  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, get_parameter("base_frame_id").as_string(), 50,
    get_node_logging_interface(),
    get_node_clock_interface(),
    tf2::durationFromSec(1.0));

  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::bind(&AmclNode::laser_callback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  particle_cloud_pub_->on_deactivate();
  likelihood_field_pub_->on_deactivate();
  pose_pub_->on_deactivate();
  laser_scan_sub_.reset();
  map_sub_.reset();
  bond_.reset();
  tf_listener_.reset();
  tf_broadcaster_.reset();
  tf_buffer_.reset();
  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  particle_cloud_pub_.reset();
  likelihood_field_pub_.reset();
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

  const auto generation_params = beluga::AdaptiveGenerationParam{
    get_parameter("recovery_alpha_slow").as_double(),
    get_parameter("recovery_alpha_fast").as_double()};

  const auto kld_resampling_params = beluga::KldResamplingParam{
    static_cast<std::size_t>(get_parameter("min_particles").as_int()),
    static_cast<std::size_t>(get_parameter("max_particles").as_int()),
    get_parameter("spatial_resolution").as_double(),
    get_parameter("pf_err").as_double(),
    get_parameter("pf_z").as_double()};

  const auto likelihood_field_model_params = beluga::LikelihoodFieldModelParam{
    get_parameter("laser_likelihood_max_dist").as_double(),
    get_parameter("laser_min_range").as_double(),
    get_parameter("laser_max_range").as_double(),
    get_parameter("z_hit").as_double(),
    get_parameter("z_rand").as_double(),
    get_parameter("sigma_hit").as_double()};

  particle_filter_ = std::make_unique<ParticleFilter>(
    generation_params,
    kld_resampling_params,
    likelihood_field_model_params,
    OccupancyGrid{map});

  RCLCPP_INFO(
    get_logger(), "Particle filter initialized with %ld particles",
    particle_filter_->particles().size());

  {
    const auto & likelihood_field = particle_filter_->likelihood_field();
    auto message = nav_msgs::msg::OccupancyGrid{};
    message.header.stamp = now();
    message.header.frame_id = map->header.frame_id;
    message.info = map->info;
    message.data.resize(likelihood_field.size());
    for (std::size_t index = 0; index < message.data.size(); ++index) {
      message.data[index] = static_cast<std::int8_t>(likelihood_field[index] * 100);
    }
    RCLCPP_INFO(get_logger(), "Publishing likelihood field");
    likelihood_field_pub_->publish(message);
  }
}

void AmclNode::timer_callback()
{
  if (!particle_filter_) {
    return;
  }

  {
    auto message = nav2_msgs::msg::ParticleCloud{};
    message.header.stamp = now();
    message.header.frame_id = get_parameter("global_frame_id").as_string();
    message.particles.resize(particle_filter_->particles().size());
    ranges::transform(
      particle_filter_->particles(), std::begin(message.particles), [](const auto & particle) {
        auto message = nav2_msgs::msg::Particle{};
        tf2::convert(beluga::state(particle), message.pose);
        message.weight = beluga::weight(particle);
        return message;
      });
    RCLCPP_INFO(get_logger(), "Publishing %ld particles", message.particles.size());
    particle_cloud_pub_->publish(message);
  }

  {
    const auto [pose, covariance] = particle_filter_->estimated_pose();
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped{};
    message.header.stamp = now();
    message.header.frame_id = get_parameter("global_frame_id").as_string();
    tf2::convert(pose, message.pose.pose);
    message.pose.covariance[0] = covariance.coeff(0, 0);
    message.pose.covariance[1] = covariance.coeff(0, 1);
    message.pose.covariance[6] = covariance.coeff(1, 0);
    message.pose.covariance[7] = covariance.coeff(1, 1);
    message.pose.covariance[35] = covariance.coeff(2, 2);
    pose_pub_->publish(message);
  }
}

void AmclNode::laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
  if (!particle_filter_) {
    return;
  }

  try {
    {
      auto base_to_laser_transform = tf2::Transform{};
      tf2::convert(
        tf_buffer_->lookupTransform(
          get_parameter("base_frame_id").as_string(),
          laser_scan->header.frame_id,
          laser_scan->header.stamp,
          std::chrono::seconds(1)).transform, base_to_laser_transform);
      const float range_min =
        std::max(
        laser_scan->range_min,
        static_cast<float>(get_parameter("laser_min_range").as_double()));
      const float range_max =
        std::min(
        laser_scan->range_max,
        static_cast<float>(get_parameter("laser_max_range").as_double()));
      auto points = std::vector<std::pair<double, double>>{};
      points.reserve(laser_scan->ranges.size());
      for (std::size_t index = 0; index < laser_scan->ranges.size(); ++index) {
        const float range = laser_scan->ranges[index];
        if (std::isnan(range) || range < range_min || range > range_max) {
          continue;
        }
        // Store points in the robot's reference frame
        const float angle = laser_scan->angle_min +
          static_cast<float>(index) * laser_scan->angle_increment;
        const auto point = base_to_laser_transform * tf2::Vector3{
          range * std::cos(angle),
          range * std::sin(angle),
          0.0};
        points.emplace_back(point.x(), point.y());
      }
      particle_filter_->update_sensor(std::move(points));
    }

    const auto time1 = std::chrono::high_resolution_clock::now();
    particle_filter_->sample();
    const auto time2 = std::chrono::high_resolution_clock::now();
    particle_filter_->importance_sample();
    const auto time3 = std::chrono::high_resolution_clock::now();
    particle_filter_->resample();
    const auto time4 = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Filter update statistics: %ld particles %ld points - %.3fms %.3fms %.3fms",
      particle_filter_->particles().size(),
      laser_scan->ranges.size(),
      std::chrono::duration<double, std::milli>(time2 - time1).count(),
      std::chrono::duration<double, std::milli>(time3 - time2).count(),
      std::chrono::duration<double, std::milli>(time4 - time3).count());
  } catch (const tf2::TransformException & error) {
    RCLCPP_ERROR(get_logger(), "Could not transform laser: %s", error.what());
  }
}

}  // namespace beluga_amcl

RCLCPP_COMPONENTS_REGISTER_NODE(beluga_amcl::AmclNode)

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

#include <chrono>
#include <limits>
#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>

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
    descriptor.description =
      "Topic to subscribe to in order to receive the map to localize on.";
    declare_parameter("map_topic", rclcpp::ParameterValue("map"), descriptor);
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

  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  particle_cloud_pub_->on_activate();
  likelihood_field_pub_->on_activate();

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

  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  particle_cloud_pub_->on_deactivate();
  likelihood_field_pub_->on_deactivate();
  map_sub_.reset();
  bond_.reset();

  return CallbackReturn::SUCCESS;
}

AmclNode::CallbackReturn AmclNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  particle_cloud_pub_.reset();
  likelihood_field_pub_.reset();
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

  auto generation_params = beluga::AdaptiveGenerationParam{
    get_parameter("recovery_alpha_slow").as_double(),
    get_parameter("recovery_alpha_fast").as_double()};

  auto kld_resampling_params = beluga::KldResamplingParam{
    static_cast<std::size_t>(get_parameter("min_particles").as_int()),
    static_cast<std::size_t>(get_parameter("max_particles").as_int()),
    get_parameter("spatial_resolution").as_double(),
    get_parameter("pf_err").as_double(),
    get_parameter("pf_z").as_double()};

  auto likelihood_model_params = LikelihoodSensorModelParam{
    get_parameter("laser_likelihood_max_dist").as_double(),
    get_parameter("laser_max_range").as_double(),
    get_parameter("z_hit").as_double(),
    get_parameter("z_rand").as_double(),
    get_parameter("sigma_hit").as_double()};

  particle_filter_ = std::make_unique<ParticleFilter>(
    generation_params, kld_resampling_params, likelihood_model_params, *map);

  RCLCPP_INFO(
    get_logger(), "Particle filter initialized with %ld particles",
    particle_filter_->particles().size());

  {
    auto message = particle_filter_->get_likelihood_field_as_gridmap();
    message.header.stamp = now();
    message.header.frame_id = get_parameter("global_frame_id").as_string();
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
        message.pose = geometry_msgs::msg::Pose{beluga::state(particle)};
        message.weight = beluga::weight(particle);
        return message;
      });
    RCLCPP_INFO(get_logger(), "Publishing %ld particles", message.particles.size());
    particle_cloud_pub_->publish(message);
  }
}

}  // namespace beluga_amcl

RCLCPP_COMPONENTS_REGISTER_NODE(beluga_amcl::AmclNode)

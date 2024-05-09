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

#include <beluga/algorithm/estimation.hpp>
#include <beluga/algorithm/spatial_hash.hpp>
#include <beluga/containers/tuple_vector.hpp>
#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/motion/stationary_model.hpp>
#include <beluga/primitives.hpp>
#include <beluga/random/multivariate_normal_distribution.hpp>
#include <beluga/sensor/ndt_sensor_model.hpp>
#include <beluga/views/particles.hpp>
#include <beluga_amcl/common.hpp>
#include <beluga_amcl/ndt_amcl_node.hpp>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/create_timer_ros.h>

#include <chrono>
#include <execution>
#include <memory>
#include <sophus/se2.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <variant>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <range/v3/algorithm/transform.hpp>
#include <range/v3/range/conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <beluga_ros/amcl.hpp>
#include <beluga_ros/tf2_sophus.hpp>

namespace beluga_amcl {

NdtAmclNode::NdtAmclNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode{"ndt_amcl", "", options} {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_common_params(*this);
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Likelihood for measurements that lie inside cells that are not present in the map.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("minimum_likelihood", rclcpp::ParameterValue(0.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Scaling parameter d1 in literature, used for scaling 2D likelihoods.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1000;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("d1", rclcpp::ParameterValue(1.0), descriptor);
  }

  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = "Scaling parameter d2 in literature, used for scaling 2D likelihoods.";
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 0;
    descriptor.floating_point_range[0].to_value = 1000;
    descriptor.floating_point_range[0].step = 0;
    declare_parameter("d2", rclcpp::ParameterValue(1.0), descriptor);
  }
}

NdtAmclNode::~NdtAmclNode() {
  RCLCPP_INFO(get_logger(), "Destroying");
  // In case this lifecycle node wasn't properly shut down, do it here
  on_shutdown(get_current_state());
}

NdtAmclNode::CallbackReturn NdtAmclNode::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Configuring");
  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>("particle_cloud", rclcpp::SensorDataQoS());
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", rclcpp::SystemDefaultsQoS());
  return CallbackReturn::SUCCESS;
}

NdtAmclNode::CallbackReturn NdtAmclNode::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Activating");
  particle_filter_ = make_particle_filter();
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
    // TODO(alon): create a parameter for the timer rate?
    timer_ = create_wall_timer(200ms, std::bind(&NdtAmclNode::timer_callback, this), common_callback_group);
  }

  {
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        get_parameter("initial_pose_topic").as_string(), rclcpp::SystemDefaultsQoS(),
        std::bind(&NdtAmclNode::initial_pose_callback, this, std::placeholders::_1), common_subscription_options);
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
        laser_scan_filter_->registerCallback(std::bind(&NdtAmclNode::laser_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to scan_topic: %s", laser_scan_sub_->getTopic().c_str());

    const auto initial_estimate = get_initial_estimate();
    if (initial_estimate.has_value()) {
      last_known_estimate_ = initial_estimate;
      initialize_from_estimate(initial_estimate.value());
    }
  }
  return CallbackReturn::SUCCESS;
}

NdtAmclNode::CallbackReturn NdtAmclNode::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Deactivating");
  particle_cloud_pub_->on_deactivate();
  pose_pub_->on_deactivate();
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

NdtAmclNode::CallbackReturn NdtAmclNode::on_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Cleaning up");
  particle_cloud_pub_.reset();
  pose_pub_.reset();
  particle_filter_.reset();
  enable_tf_broadcast_ = false;
  return CallbackReturn::SUCCESS;
}

NdtAmclNode::CallbackReturn NdtAmclNode::on_shutdown(const rclcpp_lifecycle::State& state) {
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

auto NdtAmclNode::get_initial_estimate() const -> std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> {
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

auto NdtAmclNode::get_motion_model() const -> MotionModelVariant {
  const auto name = get_parameter("robot_model_type").as_string();
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

  throw std::invalid_argument(std::string("Invalid motion model: ") + name);
}

beluga::NDTSensorModel<NDTMapRepresentation> NdtAmclNode::get_sensor_model() const {
  auto params = beluga::NDTModelParam{};
  params.minimum_likelihood = get_parameter("minimum_likelihood").as_double();
  params.d1 = get_parameter("d1").as_double();
  params.d2 = get_parameter("d2").as_double();
  const auto map_path = get_parameter("map_path").as_string();
  RCLCPP_ERROR(get_logger(), "Loading map from %s.", map_path.c_str());

  return beluga::NDTSensorModel<NDTMapRepresentation>{
      params, beluga::io::load_from_hdf5_2d<NDTMapRepresentation>(get_parameter("map_path").as_string())};
}

auto NdtAmclNode::get_execution_policy() const -> ExecutionPolicyVariant {
  const auto name = get_parameter("execution_policy").as_string();
  if (name == "seq") {
    return std::execution::seq;
  }
  if (name == "par") {
    return std::execution::par;
  }
  throw std::invalid_argument("Execution policy must be seq or par.");
};

auto NdtAmclNode::make_particle_filter() const -> std::unique_ptr<NdtAmclVariant> {
  auto amcl = std::visit(
      [this](auto motion_model, auto execution_policy) -> NdtAmclVariant {
        auto params = beluga::AmclParams{};
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

        auto hasher = beluga::spatial_hash<Sophus::SE2d>(
            get_parameter("spatial_resolution_x").as_double(), get_parameter("spatial_resolution_y").as_double(),
            get_parameter("spatial_resolution_theta").as_double());

        RandomStateGenerator random_state_maker = [](const auto& particles) {
          static thread_local auto generator = std::mt19937{std::random_device()()};
          const auto estimate = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
          return [estimate]() {
            return beluga::MultivariateNormalDistribution{estimate.first, estimate.second}(generator);
          };
        };

        return beluga::Amcl(
            std::move(motion_model),
            get_sensor_model(),             //
            std::move(random_state_maker),  //
            std::move(hasher),              //
            params,                         //
            execution_policy);
      },
      get_motion_model(), get_execution_policy());
  return std::make_unique<NdtAmclVariant>(std::move(amcl));
}

void NdtAmclNode::timer_callback() {
  if (!particle_filter_) {
    return;
  }

  if (particle_cloud_pub_->get_subscription_count() == 0) {
    return;
  }
  std::visit(
      [this](const auto& particle_filter) {
        particle_cloud_pub_->publish(
            make_representative_particle_cloud(particle_filter, get_parameter("global_frame_id").as_string(), now()));
      },
      *particle_filter_);
}

// TODO(alon): Wouldn't it be better in the callback of each message to simply receive
// it and define another timer or thread to do the work of calculation and publication?
void NdtAmclNode::laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
  if (!particle_filter_ || !last_known_estimate_.has_value()) {
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

  // TODO(nahuel, Ramiro): Remove this once we update the measurement type.
  auto scan = beluga_ros::LaserScan{
      laser_scan, laser_pose_in_base, static_cast<std::size_t>(get_parameter("max_beams").as_int()),
      get_parameter("laser_min_range").as_double(), get_parameter("laser_max_range").as_double()};
  auto measurement = scan.points_in_cartesian_coordinates() |  //
                     ranges::views::transform([&scan](const auto& p) {
                       return Eigen::Vector2d((scan.origin() * Sophus::Vector3d{p.x(), p.y(), 0}).head<2>());
                     }) |
                     ranges::to<std::vector>;
  const auto new_estimate = std::visit(
      [base_pose_in_odom, measurement = std::move(measurement)](auto& particle_filter) {
        return particle_filter.update(
            base_pose_in_odom,  //
            std::move(measurement));
      },
      *particle_filter_);
  RCLCPP_INFO(get_logger(), "No-motion update requested");

  const auto update_stop_time = std::chrono::high_resolution_clock::now();
  const auto update_duration = update_stop_time - update_start_time;

  if (new_estimate.has_value()) {
    last_known_estimate_ = new_estimate;

    const auto num_particles =
        std::visit([](const auto& particle_filter) { return particle_filter.particles().size(); }, *particle_filter_);

    RCLCPP_INFO(
        get_logger(), "Particle filter update iteration stats: %ld particles %ld points - %.3fms", num_particles,
        laser_scan->ranges.size(), std::chrono::duration<double, std::milli>(update_duration).count());
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
    const auto odom_transform_in_map = base_pose_in_map * base_pose_in_odom.inverse();
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

void NdtAmclNode::initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
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

bool NdtAmclNode::initialize_from_estimate(const std::pair<Sophus::SE2d, Eigen::Matrix3d>& estimate) {
  RCLCPP_INFO(get_logger(), "Initializing particles from estimated pose and covariance");

  if (particle_filter_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particles: The particle filter has not been initialized");
    return false;
  }

  try {
    std::visit(
        [estimate](auto& particle_filter) { particle_filter.initialize(estimate.first, estimate.second); },
        *particle_filter_);
  } catch (const std::runtime_error& error) {
    RCLCPP_ERROR(get_logger(), "Could not initialize particles: %s", error.what());
    return false;
  }

  enable_tf_broadcast_ = true;

  const auto num_particles =
      std::visit([](const auto& particle_filter) { return particle_filter.particles().size(); }, *particle_filter_);

  const auto& pose = estimate.first;
  RCLCPP_INFO(
      get_logger(), "Particle filter initialized with %ld particles about initial pose x=%g, y=%g, yaw=%g",
      num_particles, pose.translation().x(), pose.translation().y(), pose.so2().log());

  return true;
}
}  // namespace beluga_amcl

RCLCPP_COMPONENTS_REGISTER_NODE(beluga_amcl::NdtAmclNode)

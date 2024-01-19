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

#include <filesystem>
#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

#include <beluga/beluga.hpp>
#include <beluga_ros/laser_scan.hpp>
#include <beluga_ros/occupancy_grid.hpp>
#include <beluga_ros/tf2_sophus.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/generate_n.hpp>

namespace {

// ****************************************************************************
// Test utilities.
// ****************************************************************************

/// Read messages from a specific topic in a ROS bag.
template <typename Message>
auto read_messages(const std::filesystem::path& bagfile, std::string_view topic) {
  auto reader = std::make_shared<rosbag2_cpp::Reader>();  // wrapped in a shared pointer to make it copyable :)
  reader->open(bagfile.native());

  auto filter = rosbag2_storage::StorageFilter{};
  filter.topics.emplace_back(topic);
  reader->set_filter(filter);

  std::size_t size = 0UL;
  for (const auto& [topic_metadata, message_count] : reader->get_metadata().topics_with_message_count) {
    if (topic_metadata.name == topic) {
      size = message_count;
      break;
    }
  }

  return ranges::views::generate_n([reader]() mutable { return reader->read_next<Message>(); }, size);
}

/// Transform a range of odometry messages into a range of SE2 elements.
auto odometry_to_sophus() {
  return ranges::views::transform([](const nav_msgs::msg::Odometry& message) {
    Sophus::SE2d out;
    tf2::convert(message.pose.pose, out);
    return out;
  });
}

// ****************************************************************************
// Particle filter implementation variants.
// ****************************************************************************
struct StandardAMCLParams {
  double update_min_d = 0.25;
  double update_min_a = 0.2;
  std::size_t resample_interval_count = 1UL;
  bool selective_resampling = false;
  std::size_t min_particles = 500UL;
  std::size_t max_particles = 2000UL;
  double alpha_slow = 0.001;
  double alpha_fast = 0.1;
  double kld_epsilon = 0.05;
  double kld_z = 3.0;
};

// All possible particle filter configuration variants to test against.
/**
 * New particle filter implementation variants can be added by following these steps:
 *
 * - Create a new struct with the necessary configuration parameters (could be empty).
 * - Add the new struct type to this variant declaration.
 * - Add one or more instances of that struct to the `get_particle_filter_params` function.
 * - Implement a new  `particle_filter_test` overload taking that struct as a first parameter.
 */
using ParticleFilterParams = std::variant<StandardAMCLParams>;

auto get_particle_filter_params() {
  return std::vector<ParticleFilterParams>{
      StandardAMCLParams{},  // Default configuration.
      []() {
        auto params = StandardAMCLParams{};
        params.selective_resampling = true;  // Enable selective resampling.
        return params;
      }(),
  };
}

/// Overload of particle_filter_test with a standard AMCL implementation.
template <class MotionModel, class SensorModel, class Distribution, class Range>
auto particle_filter_test(
    const StandardAMCLParams& params,
    MotionModel&& motion,
    SensorModel&& sensor,
    Distribution&& initial_distribution,
    Range&& datapoints) {
  // Tolerance values ​​prove that the filter performs approximately well, they do not prove accuracy.
  // The values were adjusted to have a 100% success rate over 100 runs of the same test.
  const double position_tolerance = 0.9;
  const double orientation_tolerance = 30.0 * Sophus::Constants<double>::pi() / 180.0;

  using Particle = std::tuple<Sophus::SE2d, beluga::Weight>;

  [[maybe_unused]] static thread_local auto engine = std::mt19937{std::random_device()()};

  auto hasher = beluga::spatial_hash<Sophus::SE2d>{0.1, 0.1, 0.1};

  // Use the initial distribution to initialize particles.
  // TODO(nahuel): We should have a view to sample from an existing distribution.
  // TODO(nahuel): We should have a view to convert from Eigen to Sophus types.
  /**
   * auto particles = beluga::views::sample(initial_distribution) |
   *                  (something to convert from Eigen::Vector3d to Sophus::SE2d) |
   *                  ranges::views::transform(beluga::make_from_state<Particle>) |
   *                  ranges::views::take_exactly(params.max_particles) |
   *                  ranges::to<beluga::TupleVector>;
   */
  auto particles = ranges::views::generate([initial_distribution]() mutable {
                     const auto sample = initial_distribution(engine);
                     return Sophus::SE2d{Sophus::SO2d{sample.z()}, Eigen::Vector2d{sample.x(), sample.y()}};
                   }) |
                   ranges::views::transform(beluga::make_from_state<Particle>) |  //
                   ranges::views::take_exactly(params.max_particles) |            //
                   ranges::to<beluga::TupleVector>;

  // Don't update the filter unless we've moved far enough from the latest pose where an update was performed.
  // It's an approximation of the recommendations Probabilistic Robotics \cite thrun2005probabilistic
  // Chapter 4.2.4 based in Nav2 AMCL's implementation .
  // TODO(nahuel): This should be part of the library (in some form).
  auto far_enough_to_update = [params, latest_pose = std::optional<Sophus::SE2d>{}](auto pose) mutable -> bool {
    if (!latest_pose) {
      latest_pose = pose;
      return true;
    }

    const auto delta = latest_pose->inverse() * pose;
    const bool delta_is_above_threshold =                           //
        std::abs(delta.translation().x()) > params.update_min_d ||  //
        std::abs(delta.translation().y()) > params.update_min_d ||  //
        std::abs(delta.so2().log()) > params.update_min_a;

    if (delta_is_above_threshold) {
      latest_pose = pose;
    }

    return delta_is_above_threshold;
  };

  // TODO(nahuel): This should be part of the library (in some form).
  auto should_resample = [params, count = 0UL]() mutable -> bool {
    return ++count % params.resample_interval_count == 0;
  };

  // Iteratively run the filter through all the data points.
  for (auto [measurement, odom, ground_truth] : datapoints) {
    if (!far_enough_to_update(odom)) {
      continue;
    }

    // TODO(nahuel): Update this once the new model features are ready.
    /**
     * particles |= beluga::actions::propagate(motion(odom)) |
     *              beluga::actions::reweight(sensor(measurement));
     */
    motion.update_motion(odom);
    sensor.update_sensor(std::move(measurement));
    particles |=
        beluga::actions::propagate([&motion](const auto& state) { return motion.apply_motion(state, engine); }) |
        beluga::actions::reweight([&sensor](const auto& state) { return sensor.importance_weight(state); });

    // TODO(nahuel): Implement a `normalize` action closure that normalizes over the total weight.
    /**
     * particles |= beluga::actions::normalize;
     */
    const double total_weight = ranges::accumulate(beluga::views::weights(particles), 0.0);
    particles |= beluga::actions::reweight([total_weight](auto) { return 1.0 / total_weight; });  // HACK

    // TODO(nahuel): Implement adaptive probability estimator.
    /**
     * adaptive_probability_estimator.update(particles);
     */

    // TODO(nahuel): Sort out resampling policies.
    if (!should_resample()) {
      continue;
    }

    // Nav2 updates the filter estimates regardless of whether selective resampling actually resamples or not.
    if (!params.selective_resampling ||
        beluga::effective_sample_size(particles) < static_cast<double>(ranges::size(particles)) / 2.0) {
      // TODO(nahuel): Implement adaptive probability estimator.
      /**
       * const auto random_state_probability = adaptive_probability_estimator();
       */
      const auto random_state_probability = 0.1;

      auto make_random_particle = [&sensor] {
        return beluga::make_from_state<Particle>(sensor.make_random_state(engine));
      };

      particles |= beluga::views::sample |
                   beluga::views::random_intersperse(make_random_particle, random_state_probability) |
                   beluga::views::take_while_kld(
                       hasher, params.min_particles, params.max_particles, params.kld_epsilon, params.kld_z) |
                   beluga::actions::assign;
    }

    // TODO(nahuel): Add estimate overloads for particle ranges.
    /**
     * const auto [mean, covariance] = beluga::estimate(particles);
     */
    const auto [mean, covariance] =
        beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
    const auto error = mean.inverse() * ground_truth;
    ASSERT_LE(error.translation().norm(), position_tolerance);
    ASSERT_LE(error.so2().log(), orientation_tolerance);
  }
}

// ****************************************************************************
// Motion model variants.
// ****************************************************************************
using MotionModel = std::variant<beluga::DifferentialDriveModel>;

auto get_motion_models() {
  return std::vector<MotionModel>{
      [] {
        auto motion_params = beluga::DifferentialDriveModelParam{};
        motion_params.rotation_noise_from_rotation = 0.2;
        motion_params.rotation_noise_from_translation = 0.2;
        motion_params.translation_noise_from_translation = 0.2;
        motion_params.translation_noise_from_rotation = 0.2;
        return beluga::DifferentialDriveModel{motion_params};
      }(),
  };
}

// ****************************************************************************
// Sensor model variants.
// ****************************************************************************
using SensorModel = std::variant<
    beluga::LikelihoodFieldModel<beluga_ros::OccupancyGrid>,
    beluga::BeamSensorModel<beluga_ros::OccupancyGrid>>;

using SensorModelBuilder = std::function<SensorModel(std::shared_ptr<nav_msgs::msg::OccupancyGrid>)>;

auto get_sensor_model_builders() {
  return std::vector<SensorModelBuilder>{
      [](auto map) -> SensorModel {
        auto sensor_params = beluga::LikelihoodFieldModelParam{};
        sensor_params.max_obstacle_distance = 2.0;
        sensor_params.max_laser_distance = 100.0;
        sensor_params.z_hit = 0.5;
        sensor_params.z_random = 0.5;
        sensor_params.sigma_hit = 0.2;
        return beluga::LikelihoodFieldModel{sensor_params, beluga_ros::OccupancyGrid{std::move(map)}};
      },
      [](auto map) -> SensorModel {
        auto sensor_params = beluga::BeamModelParam{};
        sensor_params.beam_max_range = 100.0;
        return beluga::BeamSensorModel{sensor_params, beluga_ros::OccupancyGrid{std::move(map)}};
      }};
}

// ****************************************************************************
// Test cases.
// ****************************************************************************
class ParticleFilterTest
    : public testing::TestWithParam<std::tuple<ParticleFilterParams, MotionModel, SensorModelBuilder>> {};

/// Get ROS bag perfect odometry data to be used in the tests.
auto get_perfect_odometry_data() {
  constexpr std::string_view bagfile = "./bags/perfect_odometry";
  constexpr std::string_view map_topic = "/map";
  constexpr std::string_view scan_topic = "/scan";
  constexpr std::string_view odom_topic = "/odometry/ground_truth";
  constexpr std::string_view grount_truth_topic = "/odometry/ground_truth";

  auto maps = read_messages<nav_msgs::msg::OccupancyGrid>(bagfile, map_topic);
  EXPECT_EQ(maps.size(), 1) << "Expected map topic [" << map_topic << "] to have 1 message, got: " << maps.size();
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*maps.begin());

  auto scan_to_measurement = ranges::views::transform([](sensor_msgs::msg::LaserScan msg) {
    const auto laser_transform = Sophus::SE3d{Eigen::Quaterniond{1., 0., 0., 0.}, Eigen::Vector3d{0.28, 0., 0.}};
    return beluga_ros::LaserScan{
               std::make_shared<sensor_msgs::msg::LaserScan>(msg),
               laser_transform,
               60,    // max beam count
               0.,    // range min
               100.,  // range max
           }          // TODO(nahuel): Remove the transform once sensor models accept LaserScan measurements directly.
               .points_in_cartesian_coordinates() |
           ranges::views::transform([&laser_transform](const auto& p) {
             const auto result = laser_transform * Sophus::Vector3d{p.x(), p.y(), 0};
             return std::make_pair(result.x(), result.y());
           }) |
           ranges::to<std::vector>;
  });

  auto odometry = read_messages<nav_msgs::msg::Odometry>(bagfile, odom_topic) | odometry_to_sophus();
  auto ground_truth = read_messages<nav_msgs::msg::Odometry>(bagfile, grount_truth_topic) | odometry_to_sophus();
  auto measurements = read_messages<sensor_msgs::msg::LaserScan>(bagfile, scan_topic) | scan_to_measurement;

  EXPECT_EQ(measurements.size(), odometry.size());
  EXPECT_EQ(measurements.size(), ground_truth.size());

  auto datapoints = ranges::views::zip(measurements, odometry, ground_truth);

  auto initial_distribution = beluga::MultivariateNormalDistribution{
      Eigen::Vector3d{0.0, 2.0, 0.0},                                          // initial pose mean
      Eigen::Matrix3d{{0.125, 0.0, 0.0}, {0.0, 0.125, 0.0}, {0.0, 0.0, 0.04}}  // initial pose covariance
  };

  return std::make_tuple(map, datapoints, initial_distribution);
}

TEST_P(ParticleFilterTest, PerfectOdometryEstimatedPath) {
  auto [map, datapoints, distribution] = get_perfect_odometry_data();
  auto [particle_filter_params, motion_model, sensor_model_builder] = GetParam();
  auto sensor_model = sensor_model_builder(map);
  std::visit(
      [distribution = std::move(distribution), datapoints = std::move(datapoints)]  //
      (auto particle_filter_params, auto motion_model, auto sensor_model) mutable {
        particle_filter_test(
            particle_filter_params,   //
            std::move(motion_model),  //
            std::move(sensor_model),  //
            std::move(distribution),  //
            std::move(datapoints));
      },
      particle_filter_params, std::move(motion_model), std::move(sensor_model));
}

INSTANTIATE_TEST_SUITE_P(
    BagFileTest,
    ParticleFilterTest,
    testing::Combine(
        testing::ValuesIn(get_particle_filter_params()),  //
        testing::ValuesIn(get_motion_models()),           //
        testing::ValuesIn(get_sensor_model_builders())));

}  // namespace

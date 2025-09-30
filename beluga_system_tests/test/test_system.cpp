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

#include <cstddef>
#include <execution>
#include <filesystem>
#include <functional>
#include <memory>
#include <string_view>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include <tf2/convert.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/common.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

#include <range/v3/view/generate_n.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/random/multivariate_normal_distribution.hpp>
#include <beluga/sensor/beam_model.hpp>
#include <beluga/sensor/likelihood_field_model.hpp>

#include <beluga_ros/amcl.hpp>
#include <beluga_ros/laser_scan.hpp>
#include <beluga_ros/occupancy_grid.hpp>
#include <beluga_ros/tf2_sophus.hpp>

#include <rosbag2_storage/storage_filter.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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

// All possible particle filter configuration variants to test against.
/**
 * New particle filter implementation variants can be added by following these steps:
 *
 * - Create a new struct with the necessary configuration parameters (could be empty).
 * - Add the new struct type to this variant declaration.
 * - Add one or more instances of that struct to the `get_particle_filter_params` function.
 * - Implement a new  `particle_filter_test` overload taking that struct as a first parameter.
 */
using ParticleFilterParams = std::variant<beluga_ros::AmclParams>;

auto get_particle_filter_params() {
  return std::vector<ParticleFilterParams>{
      beluga_ros::AmclParams{},  // Default configuration.
      []() {
        auto params = beluga_ros::AmclParams{};
        params.selective_resampling = true;  // Enable selective resampling.
        return params;
      }(),
  };
}

/// Overload of particle_filter_test with a standard AMCL implementation.
template <class Map, class MotionModel, class SensorModel, class Distribution, class Range>
void particle_filter_test(
    const beluga_ros::AmclParams& params,
    Map&& map,
    MotionModel&& motion_model,
    SensorModel&& sensor_model,
    Distribution&& initial_distribution,
    Range&& datapoints) {
  auto filter = beluga_ros::Amcl{map, motion_model, sensor_model, params, std::execution::par};
  filter.initialize(std::forward<Distribution>(initial_distribution));

  // Tolerance values ​​prove that the filter performs approximately well, they do not prove accuracy.
  // The values were adjusted to have a 100% success rate over 100 runs of the same test.
  const double position_tolerance = 0.9;
  const double orientation_tolerance = 30.0 * Sophus::Constants<double>::pi() / 180.0;

  std::size_t update_count = 0;

  for (auto [measurement, odom, ground_truth] : datapoints) {
    const auto estimate = filter.update(beluga::TimeStamped<Sophus::SE2d>{std::move(odom)}, std::move(measurement));

    if (!estimate.has_value()) {
      continue;
    }

    const auto [mean, covariance] = estimate.value();
    const auto error = mean.inverse() * ground_truth;

    ASSERT_LE(error.translation().norm(), position_tolerance);
    ASSERT_LE(error.so2().log(), orientation_tolerance);

    ++update_count;
  }

  ASSERT_GE(update_count, 2) << "There were less than 2 updates to the filter";
}

// ****************************************************************************
// Motion model variants.
// ****************************************************************************
using MotionModel = std::variant<beluga::DifferentialDriveModel2d>;

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

using SensorModelBuilder = std::function<SensorModel(const beluga_ros::OccupancyGrid&)>;

auto get_sensor_model_builders() {
  return std::vector<SensorModelBuilder>{
      [](const auto& map) -> SensorModel {
        auto sensor_params = beluga::LikelihoodFieldModelParam{};
        sensor_params.max_obstacle_distance = 2.0;
        sensor_params.max_laser_distance = 100.0;
        sensor_params.z_hit = 0.5;
        sensor_params.z_random = 0.5;
        sensor_params.sigma_hit = 0.2;
        return beluga::LikelihoodFieldModel{sensor_params, map};
      },
      [](const auto& map) -> SensorModel {
        auto sensor_params = beluga::BeamModelParam{};
        sensor_params.beam_max_range = 100.0;
        return beluga::BeamSensorModel{sensor_params, map};
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
    };
  });

  auto odometry = read_messages<nav_msgs::msg::Odometry>(bagfile, odom_topic) | odometry_to_sophus();
  auto ground_truth = read_messages<nav_msgs::msg::Odometry>(bagfile, grount_truth_topic) | odometry_to_sophus();
  auto measurements = read_messages<sensor_msgs::msg::LaserScan>(bagfile, scan_topic) | scan_to_measurement;

  EXPECT_EQ(measurements.size(), odometry.size());
  EXPECT_EQ(measurements.size(), ground_truth.size());

  auto datapoints = ranges::views::zip(measurements, odometry, ground_truth);

  auto initial_distribution = beluga::MultivariateNormalDistribution{
      Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{0.0, 2.0}},                 // initial pose mean
      Eigen::Matrix3d{{0.125, 0.0, 0.0}, {0.0, 0.125, 0.0}, {0.0, 0.0, 0.04}}  // initial pose covariance
  };

  return std::make_tuple(beluga_ros::OccupancyGrid{std::move(map)}, datapoints, initial_distribution);
}

TEST_P(ParticleFilterTest, PerfectOdometryEstimatedPath) {
  auto [map, datapoints, distribution] = get_perfect_odometry_data();
  auto [particle_filter_params, motion_model, sensor_model_builder] = GetParam();
  auto sensor_model = sensor_model_builder(map);
  std::visit(
      [map = std::move(map), distribution = std::move(distribution), datapoints = std::move(datapoints)]  //
      (auto particle_filter_params, auto motion_model, auto sensor_model) mutable {
        particle_filter_test(
            particle_filter_params,   //
            std::move(map),           //
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

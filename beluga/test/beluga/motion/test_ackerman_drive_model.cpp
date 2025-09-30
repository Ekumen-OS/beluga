// Copyright 2022-2023 Ekumen, Inc.
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <random>
#include <sophus/se3.hpp>
#include <tuple>
#include <utility>

#include <Eigen/Core>
#include <sophus/common.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

#include <range/v3/view/common.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

#include "beluga/3d_embedding.hpp"
#include "beluga/motion/ackerman_drive_model.hpp"
#include "beluga/test/motion_utils.hpp"
#include "beluga/testing/sophus_matchers.hpp"

namespace {

using Constants = Sophus::Constants<double>;
using Eigen::Vector2d;
using Sophus::SE2d;
using Sophus::SO2d;

using beluga::testing::SE2Near;

using UUT = beluga::VelocityDriveModel2d;

class VelocityDriveModelTest : public ::testing::Test {
 protected:
  const UUT motion_model_{beluga::VelocityDriveModelParam{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::mt19937 generator_{std::random_device()()};
};

TEST_F(VelocityDriveModelTest, OneUpdate) {
  constexpr double kTolerance = 0.001;
  const auto base_pose_in_odom = SE2d{SO2d{Constants::pi()}, Vector2d{1.0, -2.0}};
  const auto previous_pose_in_odom = SE2d{SO2d{Constants::pi()}, Vector2d{1.0, -2.0}};
  const auto laser_scan_stamp = std::chrono::system_clock::now();
  const auto previous_stamp = laser_scan_stamp - std::chrono::milliseconds(100);
  const auto control_action =
      beluga::testing::make_control_action(base_pose_in_odom, previous_pose_in_odom, laser_scan_stamp, previous_stamp);
  const auto state_sampling_function = motion_model_(control_action);
  const auto pose = SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, 5.0}};
  ASSERT_THAT(state_sampling_function(pose, generator_), SE2Near(pose, kTolerance));
}

TEST_F(VelocityDriveModelTest, Translate) {
  constexpr double kTolerance = 0.001;
  const auto base_pose_in_odom = SE2d{SO2d{0.0}, Vector2d{1.0, 0.0}};
  const auto previous_pose_in_odom = SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}};
  const auto laser_scan_stamp = std::chrono::system_clock::now();
  const auto previous_stamp = laser_scan_stamp - std::chrono::milliseconds(100);
  const auto control_action =
      beluga::testing::make_control_action(base_pose_in_odom, previous_pose_in_odom, laser_scan_stamp, previous_stamp);
  const auto state_sampling_function = motion_model_(control_action);

  const auto result1 = state_sampling_function(SE2d{SO2d{0.0}, Vector2d{2.0, 0.0}}, generator_);
  ASSERT_THAT(result1, SE2Near(SO2d{0.0}, Vector2d{3.0, 0.0}, kTolerance));
  const auto result2 = state_sampling_function(SE2d{SO2d{0.0}, Vector2d{0.0, 3.0}}, generator_);
  ASSERT_THAT(result2, SE2Near(SO2d{0.0}, Vector2d{1.0, 3.0}, kTolerance));
}

TEST_F(VelocityDriveModelTest, ArcOfCircumference) {
  constexpr double kTolerance = 0.001;
  const auto base_pose_in_odom = SE2d{SO2d{Constants::pi() / 2}, Vector2d{0.0, 1.0}};
  const auto previous_pose_in_odom = SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}};
  const auto laser_scan_stamp = std::chrono::system_clock::now();
  const auto previous_stamp = laser_scan_stamp - std::chrono::milliseconds(100);
  const auto control_action =
      beluga::testing::make_control_action(base_pose_in_odom, previous_pose_in_odom, laser_scan_stamp, previous_stamp);
  const auto state_sampling_function = motion_model_(control_action);
  const auto result1 = state_sampling_function(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(
      result1, SE2Near(SO2d{Constants::pi() / 2}, Vector2d{std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0}, kTolerance));
  const auto result2 = state_sampling_function(SE2d{SO2d{-Constants::pi() / 2}, Vector2d{2.0, 3.0}}, generator_);
  ASSERT_THAT(
      result2, SE2Near(SO2d{0.0}, Vector2d{2.0 + std::sqrt(2.0) / 2.0, 3.0 - std::sqrt(2.0) / 2.0}, kTolerance));
}

TEST_F(VelocityDriveModelTest, Rotate) {
  constexpr double kTolerance = 0.001;
  const auto base_pose_in_odom = SE2d{SO2d{Constants::pi() / 4}, Vector2d{0.0, 0.0}};
  const auto previous_pose_in_odom = SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}};
  const auto laser_scan_stamp = std::chrono::system_clock::now();
  const auto previous_stamp = laser_scan_stamp - std::chrono::milliseconds(100);
  const auto control_action =
      beluga::testing::make_control_action(base_pose_in_odom, previous_pose_in_odom, laser_scan_stamp, previous_stamp);
  const auto state_sampling_function = motion_model_(control_action);
  const auto result1 = state_sampling_function(SE2d{SO2d{Constants::pi()}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result1, SE2Near(SO2d{Constants::pi() * 5 / 4}, Vector2d{0.0, 0.0}, kTolerance));
  const auto result2 = state_sampling_function(SE2d{SO2d{-Constants::pi() / 2}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result2, SE2Near(SO2d{-Constants::pi() / 4}, Vector2d{0.0, 0.0}, kTolerance));
}

template <class Range>
auto get_statistics(Range&& range) {
  const auto size = static_cast<double>(std::distance(std::begin(range), std::end(range)));
  const double sum = std::accumulate(std::begin(range), std::end(range), 0.0);
  const double mean = sum / size;
  const double squared_diff_sum =
      std::transform_reduce(std::begin(range), std::end(range), 0.0, std::plus<>{}, [mean](double value) {
        const double diff = value - mean;
        return diff * diff;
      });
  const double stddev = std::sqrt(squared_diff_sum / size);
  return std::pair{mean, stddev};
}

TEST(VelocityDriveModelSamples, Translate) {
  const double tolerance = 0.015;
  const double alpha = 0.2;
  const double origin = 5.0;
  const double distance = 3.0;
  const double delta_time = 0.1;
  const auto motion_model = UUT{beluga::VelocityDriveModelParam{0.0, 0.0, alpha, 0.0, 0.0, 0.0}};
  auto generator = std::mt19937{std::random_device()()};
  const auto base_pose_in_odom = SE2d{SO2d{0.0}, Vector2d{distance, 0.0}};
  const auto previous_pose_in_odom = SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}};
  const auto laser_scan_stamp = std::chrono::system_clock::now();
  const auto previous_stamp = laser_scan_stamp - std::chrono::milliseconds(100);
  const auto control_action =
      beluga::testing::make_control_action(base_pose_in_odom, previous_pose_in_odom, laser_scan_stamp, previous_stamp);
  const auto state_sampling_function = motion_model(control_action);
  auto view = ranges::views::generate([&]() {
                const auto pose = SE2d{SO2d{0.0}, Vector2d{origin, 0.0}};
                return state_sampling_function(pose, generator).translation().x();
              }) |
              ranges::views::take_exactly(100'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, origin + distance, tolerance);
  ASSERT_NEAR(stddev, std::sqrt(alpha * distance * delta_time), tolerance);
}

TEST(VelocityDriveModelSamples, RotateFirstQuadrant) {
  const double tolerance = 0.01;
  const double alpha = 0.2;
  const double initial_angle = Constants::pi() / 6;
  const double motion_angle = Constants::pi() / 4;
  const double delta_time = 0.1;
  const auto motion_model = UUT{beluga::VelocityDriveModelParam{alpha, 0.0, 0.0, 0.0, 0.0, 0.0}};
  auto generator = std::mt19937{std::random_device()()};
  const auto base_pose_in_odom = SE2d{SO2d{motion_angle}, Vector2d{0.0, 0.0}};
  const auto previous_pose_in_odom = SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}};
  const auto laser_scan_stamp = std::chrono::system_clock::now();
  const auto previous_stamp = laser_scan_stamp - std::chrono::milliseconds(100);
  const auto control_action =
      beluga::testing::make_control_action(base_pose_in_odom, previous_pose_in_odom, laser_scan_stamp, previous_stamp);
  const auto state_sampling_function = motion_model(control_action);
  auto view = ranges::views::generate([&]() {
                const auto pose = SE2d{SO2d{initial_angle}, Vector2d{0.0, 0.0}};
                return state_sampling_function(pose, generator).so2().log();
              }) |
              ranges::views::take_exactly(100'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, initial_angle + motion_angle, tolerance);
  ASSERT_NEAR(stddev, std::sqrt(alpha * motion_angle * delta_time), tolerance);
}

TEST(VelocityDriveModelSamples, RotateThirdQuadrant) {
  const double tolerance = 0.01;
  const double alpha = 0.2;
  const double initial_angle = Constants::pi() / 6;
  const double motion_angle = -Constants::pi() * 3 / 4;
  const double delta_time = 0.1;
  const auto motion_model = UUT{beluga::VelocityDriveModelParam{alpha, 0.0, 0.0, 0.0, 0.0, 0.0}};
  auto generator = std::mt19937{std::random_device()()};
  const auto base_pose_in_odom = SE2d{SO2d{motion_angle}, Vector2d{0.0, 0.0}};
  const auto previous_pose_in_odom = SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}};
  const auto laser_scan_stamp = std::chrono::system_clock::now();
  const auto previous_stamp = laser_scan_stamp - std::chrono::milliseconds(100);
  const auto control_action =
      beluga::testing::make_control_action(base_pose_in_odom, previous_pose_in_odom, laser_scan_stamp, previous_stamp);
  const auto state_sampling_function = motion_model(control_action);
  auto view = ranges::views::generate([&]() {
                const auto pose = SE2d{SO2d{initial_angle}, Vector2d{0.0, 0.0}};
                return state_sampling_function(pose, generator).so2().log();
              }) |
              ranges::views::take_exactly(100'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, initial_angle + motion_angle, tolerance);

  ASSERT_NEAR(stddev, std::sqrt(alpha * std::abs(motion_angle) * delta_time), tolerance);
}

TEST(VelocityDriveModelSamples, ArcOfCircumference) {
  const double tolerance = 0.015;
  const double alpha = 0.2;
  const double delta_time = 0.1;
  const auto motion_model = UUT{beluga::VelocityDriveModelParam{0.0, 0.0, 0.0, alpha, 0.0, 0.0}};
  auto generator = std::mt19937{std::random_device()()};
  const auto base_pose_in_odom = SE2d{SO2d{Constants::pi() / 2}, Vector2d{0.0, 2.0}};
  const auto previous_pose_in_odom = SE2d{SO2d{0.0}, Vector2d{2.0, 0.0}};
  const auto laser_scan_stamp = std::chrono::system_clock::now();
  const auto previous_stamp = laser_scan_stamp - std::chrono::milliseconds(100);
  const auto control_action =
      beluga::testing::make_control_action(base_pose_in_odom, previous_pose_in_odom, laser_scan_stamp, previous_stamp);
  const auto state_sampling_function = motion_model(control_action);

  auto view = ranges::views::generate([&]() {
                const auto pose = SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}};
                return state_sampling_function(pose, generator).translation().norm();
              }) |
              ranges::views::take_exactly(100'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);

  ASSERT_NEAR(mean, 2.0 * std::sqrt(2.0), tolerance);

  const double angular_velocity = (Constants::pi() / 2) / delta_time;
  ASSERT_NEAR(stddev, std::sqrt(2.0 * alpha / std::abs(angular_velocity)), tolerance);
}
}  // namespace

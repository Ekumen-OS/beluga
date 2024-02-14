// Copyright 2023 Ekumen, Inc.
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

#include <beluga/motion.hpp>
#include <beluga/testing.hpp>

#include <range/v3/view/common.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

namespace {

using Constants = Sophus::Constants<double>;
using Eigen::Vector2d;
using Sophus::SE2d;
using Sophus::SO2d;

using beluga::testing::SE2Near;
using UUT = beluga::OmnidirectionalDriveModel;

class OmnidirectionalDriveModelTest : public ::testing::Test {
 protected:
  const UUT motion_model_{beluga::OmnidirectionalDriveModelParam{0.0, 0.0, 0.0, 0.0, 0.0}};  // No variance
  std::mt19937 generator_{std::random_device()()};
};

TEST_F(OmnidirectionalDriveModelTest, OneUpdate) {
  constexpr double kTolerance = 0.001;
  const auto control_action = std::make_tuple(
      SE2d{SO2d{Constants::pi()}, Vector2d{1.0, -2.0}}, SE2d{SO2d{Constants::pi()}, Vector2d{1.0, -2.0}});
  const auto state_sampling_function = motion_model_(control_action);
  const auto pose = SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, 5.0}};
  ASSERT_THAT(state_sampling_function(pose, generator_), SE2Near(pose, kTolerance));
}

TEST_F(OmnidirectionalDriveModelTest, Translate) {
  constexpr double kTolerance = 0.001;
  const auto control_action = std::make_tuple(SE2d{SO2d{0.0}, Vector2d{1.0, 0.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  const auto state_sampling_function = motion_model_(control_action);
  const auto result1 = state_sampling_function(SE2d{SO2d{0.0}, Vector2d{2.0, 0.0}}, generator_);
  ASSERT_THAT(result1, SE2Near(SO2d{0.0}, Vector2d{3.0, 0.0}, kTolerance));
  const auto result2 = state_sampling_function(SE2d{SO2d{0.0}, Vector2d{0.0, 3.0}}, generator_);
  ASSERT_THAT(result2, SE2Near(SO2d{0.0}, Vector2d{1.0, 3.0}, kTolerance));
}

TEST_F(OmnidirectionalDriveModelTest, RotateTranslate) {
  constexpr double kTolerance = 0.001;
  const auto control_action =
      std::make_tuple(SE2d{SO2d{Constants::pi() / 2}, Vector2d{0.0, 1.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  const auto state_sampling_function = motion_model_(control_action);
  const auto result1 = state_sampling_function(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result1, SE2Near(SO2d{Constants::pi() / 2}, Vector2d{0.0, 1.0}, kTolerance));
  const auto result2 = state_sampling_function(SE2d{SO2d{-Constants::pi() / 2}, Vector2d{2.0, 3.0}}, generator_);
  ASSERT_THAT(result2, SE2Near(SO2d{0.0}, Vector2d{3.0, 3.0}, kTolerance));
}

TEST_F(OmnidirectionalDriveModelTest, Rotate) {
  constexpr double kTolerance = 0.001;
  const auto control_action =
      std::make_tuple(SE2d{SO2d{Constants::pi() / 4}, Vector2d{0.0, 0.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  const auto state_sampling_function = motion_model_(control_action);
  const auto result1 = state_sampling_function(SE2d{SO2d{Constants::pi()}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result1, SE2Near(SO2d{Constants::pi() * 5 / 4}, Vector2d{0.0, 0.0}, kTolerance));
  const auto result2 = state_sampling_function(SE2d{SO2d{-Constants::pi() / 2}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result2, SE2Near(SO2d{-Constants::pi() / 4}, Vector2d{0.0, 0.0}, kTolerance));
}

TEST_F(OmnidirectionalDriveModelTest, TranslateStrafe) {
  constexpr double kTolerance = 0.001;
  const auto control_action = std::make_tuple(SE2d{SO2d{0.0}, Vector2d{0.0, 1.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  const auto state_sampling_function = motion_model_(control_action);
  const auto result1 = state_sampling_function(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result1, SE2Near(SO2d{0.0}, Vector2d{0.0, 1.0}, kTolerance));
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

TEST(OmnidirectionalDriveModelSamples, Translate) {
  const double tolerance = 0.01;
  const double alpha = 0.2;
  const double origin = 5.0;
  const double distance = 3.0;
  const auto motion_model =
      UUT{beluga::OmnidirectionalDriveModelParam{0.0, 0.0, alpha, 0.0, 0.0}};  // Translation variance
  auto generator = std::mt19937{std::random_device()()};
  const auto control_action =
      std::make_tuple(SE2d{SO2d{0.0}, Vector2d{distance, 0.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  const auto state_sampling_function = motion_model(control_action);
  auto view = ranges::views::generate([&]() {
                const auto pose = SE2d{SO2d{0.0}, Vector2d{origin, 0.0}};
                return state_sampling_function(pose, generator).translation().x();
              }) |
              ranges::views::take_exactly(1'000'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, origin + distance, tolerance);
  ASSERT_NEAR(stddev, std::sqrt(alpha * distance * distance), tolerance);
}

TEST(OmnidirectionalDriveModelSamples, RotateFirstQuadrant) {
  const double tolerance = 0.01;
  const double alpha = 0.2;
  const double initial_angle = Constants::pi() / 6;
  const double motion_angle = Constants::pi() / 4;
  const auto motion_model = UUT{beluga::OmnidirectionalDriveModelParam{alpha, 0.0, 0.0, 0.0, 0.0}};
  auto generator = std::mt19937{std::random_device()()};
  const auto control_action =
      std::make_tuple(SE2d{SO2d{motion_angle}, Vector2d{0.0, 0.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  const auto state_sampling_function = motion_model(control_action);
  auto view = ranges::views::generate([&]() {
                const auto pose = SE2d{SO2d{initial_angle}, Vector2d{0.0, 0.0}};
                return state_sampling_function(pose, generator).so2().log();
              }) |
              ranges::views::take_exactly(1'000'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, initial_angle + motion_angle, tolerance);
  ASSERT_NEAR(stddev, std::sqrt(alpha * motion_angle * motion_angle), tolerance);
}

TEST(OmnidirectionalDriveModelSamples, RotateThirdQuadrant) {
  const double tolerance = 0.01;
  const double alpha = 0.2;
  const double initial_angle = Constants::pi() / 6;
  const double motion_angle = -Constants::pi() * 3 / 4;
  const auto motion_model = UUT{beluga::OmnidirectionalDriveModelParam{alpha, 0.0, 0.0, 0.0, 0.0}};
  auto generator = std::mt19937{std::random_device()()};
  const auto control_action =
      std::make_tuple(SE2d{SO2d{motion_angle}, Vector2d{0.0, 0.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  const auto state_sampling_function = motion_model(control_action);
  auto view = ranges::views::generate([&]() {
                const auto pose = SE2d{SO2d{initial_angle}, Vector2d{0.0, 0.0}};
                return state_sampling_function(pose, generator).so2().log();
              }) |
              ranges::views::take_exactly(1'000'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, initial_angle + motion_angle, tolerance);

  // Treat backward and forward motion symmetrically for the noise models.
  const double flipped_angle = Constants::pi() + motion_angle;
  ASSERT_NEAR(stddev, std::sqrt(alpha * flipped_angle * flipped_angle), tolerance);
}

}  // namespace

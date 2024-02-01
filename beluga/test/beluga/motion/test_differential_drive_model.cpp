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

#include <beluga/motion.hpp>
#include <beluga/testing.hpp>

#include <ciabatta/ciabatta.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

namespace {

using Constants = Sophus::Constants<double>;
using Eigen::Vector2d;
using Sophus::SE2d;
using Sophus::SO2d;

using beluga::testing::SE2Near;

using UUT = beluga::DifferentialDriveModel;

class DifferentialDriveModelTest : public ::testing::Test {
 protected:
  UUT model_{beluga::DifferentialDriveModelParam{0.0, 0.0, 0.0, 0.0}};  // No variance
  std::mt19937 generator_{std::random_device()()};
};

TEST_F(DifferentialDriveModelTest, NoUpdate) {
  const auto pose = SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, 5.0}};
  ASSERT_THAT(model_.apply_motion(pose, generator_), SE2Near(pose, 0.001));
}

TEST_F(DifferentialDriveModelTest, OneUpdate) {
  const auto pose = SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, 5.0}};
  model_.update_motion(SE2d{SO2d{Constants::pi()}, Vector2d{1.0, -2.0}});
  ASSERT_THAT(model_.apply_motion(pose, generator_), SE2Near(pose, 0.001));
}

TEST_F(DifferentialDriveModelTest, Translate) {
  model_.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  model_.update_motion(SE2d{SO2d{0.0}, Vector2d{1.0, 0.0}});
  const auto result1 = model_.apply_motion(SE2d{SO2d{0.0}, Vector2d{2.0, 0.0}}, generator_);
  ASSERT_THAT(result1, SE2Near(SO2d{0.0}, Vector2d{3.0, 0.0}, 0.001));
  const auto result2 = model_.apply_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 3.0}}, generator_);
  ASSERT_THAT(result2, SE2Near(SO2d{0.0}, Vector2d{1.0, 3.0}, 0.001));
}

TEST_F(DifferentialDriveModelTest, RotateTranslate) {
  model_.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  model_.update_motion(SE2d{SO2d{Constants::pi() / 2}, Vector2d{0.0, 1.0}});
  const auto result1 = model_.apply_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result1, SE2Near(SO2d{Constants::pi() / 2}, Vector2d{0.0, 1.0}, 0.001));
  const auto result2 = model_.apply_motion(SE2d{SO2d{-Constants::pi() / 2}, Vector2d{2.0, 3.0}}, generator_);
  ASSERT_THAT(result2, SE2Near(SO2d{0.0}, Vector2d{3.0, 3.0}, 0.001));
}

TEST_F(DifferentialDriveModelTest, Rotate) {
  model_.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  model_.update_motion(SE2d{SO2d{Constants::pi() / 4}, Vector2d{0.0, 0.0}});
  const auto result1 = model_.apply_motion(SE2d{SO2d{Constants::pi()}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result1, SE2Near(SO2d{Constants::pi() * 5 / 4}, Vector2d{0.0, 0.0}, 0.001));
  const auto result2 = model_.apply_motion(SE2d{SO2d{-Constants::pi() / 2}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result2, SE2Near(SO2d{-Constants::pi() / 4}, Vector2d{0.0, 0.0}, 0.001));
}

TEST_F(DifferentialDriveModelTest, RotateTranslateRotate) {
  model_.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  model_.update_motion(SE2d{SO2d{-Constants::pi() / 2}, Vector2d{1.0, 2.0}});
  const auto result = model_.apply_motion(SE2d{SO2d{Constants::pi()}, Vector2d{3.0, 4.0}}, generator_);
  ASSERT_THAT(result, SE2Near(SO2d{Constants::pi() / 2}, Vector2d{2.0, 2.0}, 0.001));
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

TEST(DifferentialDriveModelSamples, Translate) {
  const double alpha = 0.2;
  const double origin = 5.0;
  const double distance = 3.0;
  auto model = UUT{beluga::DifferentialDriveModelParam{0.0, 0.0, alpha, 0.0}};  // Translation variance
  auto generator = std::mt19937{std::random_device()()};
  model.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  model.update_motion(SE2d{SO2d{0.0}, Vector2d{distance, 0.0}});
  auto view = ranges::views::generate([&]() {
                return model.apply_motion(SE2d{SO2d{0.0}, Vector2d{origin, 0.0}}, generator).translation().x();
              }) |
              ranges::views::take_exactly(1'000'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, origin + distance, 0.01);
  ASSERT_NEAR(stddev, std::sqrt(alpha * distance * distance), 0.01);
}

TEST(DifferentialDriveModelSamples, RotateFirstQuadrant) {
  const double alpha = 0.2;
  const double initial_angle = Constants::pi() / 6;
  const double motion_angle = Constants::pi() / 4;
  auto model = UUT{beluga::DifferentialDriveModelParam{alpha, 0.0, 0.0, 0.0}};  // Rotation variance
  auto generator = std::mt19937{std::random_device()()};
  model.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  model.update_motion(SE2d{SO2d{motion_angle}, Vector2d{0.0, 0.0}});
  auto view = ranges::views::generate([&]() {
                return model.apply_motion(SE2d{SO2d{initial_angle}, Vector2d{0.0, 0.0}}, generator).so2().log();
              }) |
              ranges::views::take_exactly(1'000'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, initial_angle + motion_angle, 0.01);
  ASSERT_NEAR(stddev, std::sqrt(alpha * motion_angle * motion_angle), 0.01);
}

TEST(DifferentialDriveModelSamples, RotateThirdQuadrant) {
  const double alpha = 0.2;
  const double initial_angle = Constants::pi() / 6;
  const double motion_angle = -Constants::pi() * 3 / 4;
  auto model = UUT{beluga::DifferentialDriveModelParam{alpha, 0.0, 0.0, 0.0}};  // Rotation variance
  auto generator = std::mt19937{std::random_device()()};
  model.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  model.update_motion(SE2d{SO2d{motion_angle}, Vector2d{0.0, 0.0}});
  auto view = ranges::views::generate([&]() {
                return model.apply_motion(SE2d{SO2d{initial_angle}, Vector2d{0.0, 0.0}}, generator).so2().log();
              }) |
              ranges::views::take_exactly(1'000'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, initial_angle + motion_angle, 0.01);

  // Treat backward and forward motion symmetrically for the noise models.
  const double flipped_angle = Constants::pi() + motion_angle;
  ASSERT_NEAR(stddev, std::sqrt(alpha * flipped_angle * flipped_angle), 0.01);
}

TEST(DifferentialDriveModelSamples, RotateTranslateRotateFirstQuadrant) {
  const double alpha = 0.2;
  auto model = UUT{beluga::DifferentialDriveModelParam{0.0, 0.0, 0.0, alpha}};  // Translation variance from rotation
  auto generator = std::mt19937{std::random_device()()};
  model.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  model.update_motion(SE2d{SO2d{0.0}, Vector2d{1.0, 1.0}});
  auto view = ranges::views::generate([&]() {
                return model.apply_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}, generator).translation().norm();
              }) |
              ranges::views::take_exactly(1'000'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, 1.41, 0.01);

  // Net rotation is zero comparing final and initial poses, but
  // the model requires a 45 degree counter-clockwise rotation,
  // a forward translation, and a 45 degree clockwise rotation.
  const double first_rotation = Constants::pi() / 4;
  const double second_rotation = first_rotation;
  const double rotation_variance = (first_rotation * first_rotation) + (second_rotation * second_rotation);
  ASSERT_NEAR(stddev, std::sqrt(alpha * rotation_variance), 0.01);
}

TEST(DifferentialDriveModelSamples, RotateTranslateRotateThirdQuadrant) {
  const double alpha = 0.2;
  auto model = UUT{beluga::DifferentialDriveModelParam{0.0, 0.0, 0.0, alpha}};  // Translation variance from rotation
  auto generator = std::mt19937{std::random_device()()};
  model.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  model.update_motion(SE2d{SO2d{0.0}, Vector2d{-1.0, -1.0}});
  auto view = ranges::views::generate([&]() {
                return model.apply_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}, generator).translation().norm();
              }) |
              ranges::views::take_exactly(1'000'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, 1.41, 0.01);

  // Net rotation is zero comparing final and initial poses, but
  // the model requires a 45 degree counter-clockwise rotation,
  // a backward translation and a 45 degree clockwise rotation.
  const double first_rotation = Constants::pi() / 4;
  const double second_rotation = first_rotation;
  const double rotation_variance = (first_rotation * first_rotation) + (second_rotation * second_rotation);
  ASSERT_NEAR(stddev, std::sqrt(alpha * rotation_variance), 0.01);
}

}  // namespace

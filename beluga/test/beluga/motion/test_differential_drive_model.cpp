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

#include <gmock/gmock.h>

#include <beluga/motion.hpp>
#include <ciabatta/ciabatta.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

#include "../../utils/sophus_matchers.hpp"

namespace {

using Constants = Sophus::Constants<double>;
using Eigen::Vector2d;
using Sophus::SE2d;
using Sophus::SO2d;

using UUT =
    ciabatta::mixin<beluga::DifferentialDriveModel, ciabatta::provides<beluga::OdometryMotionModelInterface2d>::mixin>;

class DifferentialDriveModelTest : public ::testing::Test {
 protected:
  UUT mixin_{beluga::DifferentialDriveModelParam{0.0, 0.0, 0.0, 0.0}};  // No variance
  std::mt19937 generator_{std::random_device()()};
};

TEST_F(DifferentialDriveModelTest, NoUpdate) {
  const auto pose = SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, 5.0}};
  ASSERT_THAT(mixin_.apply_motion(pose, generator_), testing::SE2Eq(pose));
}

TEST_F(DifferentialDriveModelTest, OneUpdate) {
  const auto pose = SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, 5.0}};
  mixin_.update_motion(SE2d{SO2d{Constants::pi()}, Vector2d{1.0, -2.0}});
  ASSERT_THAT(mixin_.apply_motion(pose, generator_), testing::SE2Eq(pose));
}

TEST_F(DifferentialDriveModelTest, Translate) {
  mixin_.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  mixin_.update_motion(SE2d{SO2d{0.0}, Vector2d{1.0, 0.0}});
  const auto result1 = mixin_.apply_motion(SE2d{SO2d{0.0}, Vector2d{2.0, 0.0}}, generator_);
  ASSERT_THAT(result1, testing::SE2Eq(SO2d{0.0}, Vector2d{3.0, 0.0}));
  const auto result2 = mixin_.apply_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 3.0}}, generator_);
  ASSERT_THAT(result2, testing::SE2Eq(SO2d{0.0}, Vector2d{1.0, 3.0}));
}

TEST_F(DifferentialDriveModelTest, RotateTranslate) {
  mixin_.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  mixin_.update_motion(SE2d{SO2d{Constants::pi() / 2}, Vector2d{0.0, 1.0}});
  const auto result1 = mixin_.apply_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result1, testing::SE2Eq(SO2d{Constants::pi() / 2}, Vector2d{0.0, 1.0}));
  const auto result2 = mixin_.apply_motion(SE2d{SO2d{-Constants::pi() / 2}, Vector2d{2.0, 3.0}}, generator_);
  ASSERT_THAT(result2, testing::SE2Eq(SO2d{0.0}, Vector2d{3.0, 3.0}));
}

TEST_F(DifferentialDriveModelTest, Rotate) {
  mixin_.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  mixin_.update_motion(SE2d{SO2d{Constants::pi() / 4}, Vector2d{0.0, 0.0}});
  const auto result1 = mixin_.apply_motion(SE2d{SO2d{Constants::pi()}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result1, testing::SE2Eq(SO2d{Constants::pi() * 5 / 4}, Vector2d{0.0, 0.0}));
  const auto result2 = mixin_.apply_motion(SE2d{SO2d{-Constants::pi() / 2}, Vector2d{0.0, 0.0}}, generator_);
  ASSERT_THAT(result2, testing::SE2Eq(SO2d{-Constants::pi() / 4}, Vector2d{0.0, 0.0}));
}

TEST_F(DifferentialDriveModelTest, RotateTranslateRotate) {
  mixin_.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  mixin_.update_motion(SE2d{SO2d{-Constants::pi() / 2}, Vector2d{1.0, 2.0}});
  const auto result = mixin_.apply_motion(SE2d{SO2d{Constants::pi()}, Vector2d{3.0, 4.0}}, generator_);
  ASSERT_THAT(result, testing::SE2Eq(SO2d{Constants::pi() / 2}, Vector2d{2.0, 2.0}));
}

template <class Range>
auto get_statistics(Range&& range) {
  const double size = static_cast<double>(std::distance(std::begin(range), std::end(range)));
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
  auto mixin = UUT{beluga::DifferentialDriveModelParam{0.0, 0.0, alpha, 0.0}};  // Translation variance
  auto generator = std::mt19937{std::random_device()()};
  mixin.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  mixin.update_motion(SE2d{SO2d{0.0}, Vector2d{distance, 0.0}});
  auto view = ranges::view::generate([&]() {
                return mixin.apply_motion(SE2d{SO2d{0.0}, Vector2d{origin, 0.0}}, generator).translation().x();
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
  auto mixin = UUT{beluga::DifferentialDriveModelParam{alpha, 0.0, 0.0, 0.0}};  // Rotation variance
  auto generator = std::mt19937{std::random_device()()};
  mixin.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  mixin.update_motion(SE2d{SO2d{motion_angle}, Vector2d{0.0, 0.0}});
  auto view = ranges::view::generate([&]() {
                return mixin.apply_motion(SE2d{SO2d{initial_angle}, Vector2d{0.0, 0.0}}, generator).so2().log();
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
  auto mixin = UUT{beluga::DifferentialDriveModelParam{alpha, 0.0, 0.0, 0.0}};  // Rotation variance
  auto generator = std::mt19937{std::random_device()()};
  mixin.update_motion(SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}});
  mixin.update_motion(SE2d{SO2d{motion_angle}, Vector2d{0.0, 0.0}});
  auto view = ranges::view::generate([&]() {
                return mixin.apply_motion(SE2d{SO2d{initial_angle}, Vector2d{0.0, 0.0}}, generator).so2().log();
              }) |
              ranges::views::take_exactly(1'000'000) | ranges::views::common;
  const auto [mean, stddev] = get_statistics(view);
  ASSERT_NEAR(mean, initial_angle + motion_angle, 0.01);

  // Treat backward and forward motion symmetrically for the noise models.
  const double flipped_angle = Constants::pi() + motion_angle;
  ASSERT_NEAR(stddev, std::sqrt(alpha * flipped_angle * flipped_angle), 0.01);
}

TEST(DifferentialDriveModelSamples, LatestMotionUpdateWorks) {
  // tests that the latest motion update can be recovered
  auto uut = UUT{beluga::DifferentialDriveModelParam{1.0, 0.0, 0.0, 0.0}};

  auto make_motion_update = [](double x, double y, double phi) {
    using Eigen::Vector2d;
    using Sophus::SE2d;
    using Sophus::SO2d;
    return Sophus::SE2d{SO2d{phi}, Vector2d{x, y}};
  };

  {
    const auto lmu = uut.latest_motion_update();
    ASSERT_FALSE(lmu);
  }

  {
    const auto expected_motion_update = make_motion_update(0.1, 0.2, 0.3);
    uut.update_motion(expected_motion_update);
    const auto latest_motion_update = uut.latest_motion_update();
    ASSERT_TRUE(latest_motion_update);
    ASSERT_THAT(expected_motion_update, testing::SE2Eq(latest_motion_update.value()));
  }

  {
    const auto expected_motion_update = make_motion_update(0.4, 0.5, 0.6);
    uut.update_motion(expected_motion_update);
    const auto latest_motion_update = uut.latest_motion_update();
    ASSERT_TRUE(latest_motion_update);
    ASSERT_THAT(expected_motion_update, testing::SE2Eq(latest_motion_update.value()));
  }
}

}  // namespace

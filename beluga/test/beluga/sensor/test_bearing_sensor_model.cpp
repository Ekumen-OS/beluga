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

#include <gtest/gtest.h>

// standard library
#include <cstdint>
#include <functional>
#include <numeric>
#include <tuple>
#include <utility>
#include <vector>

// external
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

#include <sophus/common.hpp>
#include <sophus/se2.hpp>

// project
#include <beluga/sensor/data/landmark_map.hpp>
#include "beluga/sensor/bearing_sensor_model.hpp"
#include "beluga/types/landmark_detection_types.hpp"

namespace beluga {

using Sensor2D = beluga::BearingSensorModel2d<LandmarkMap>;
using Sensor3D = beluga::BearingSensorModel3d<LandmarkMap>;

LandmarkMapBoundaries default_map_boundaries{Eigen::Vector3d{-10.0, -10.0, 0.0}, Eigen::Vector3d{10.0, 10.0, 0.0}};

double expected_aggregate_probability(std::vector<double> landmark_probs) {
  return std::transform_reduce(
      landmark_probs.cbegin(), landmark_probs.cend(), 1.0, std::multiplies{}, [](const double v) { return v; });
}

BearingModelParam get_default_model_params() {
  BearingModelParam ret;
  ret.sigma_bearing = Sophus::Constants<double>::pi() / 4.0;  // 45 degrees
  ret.sensor_pose_in_robot = Sophus::SE3d{Sophus::SO3d{}, Eigen::Vector3d{0.0, 0.0, 1.0}};
  return ret;
}

auto make_sensor_data(std::vector<std::tuple<double, double, double, uint32_t>> detections) {
  const auto conversion_function = [](const auto& t) {
    const auto [x, y, z, category] = t;
    return LandmarkBearingDetection{Eigen::Vector3d{x, y, z}, category};
  };
  return detections | ranges::views::transform(conversion_function) | ranges::to<std::vector>;
}

template <typename T>
T get_robot_pose_in_world();

template <>
Sophus::SE2d get_robot_pose_in_world<Sophus::SE2d>() {
  return Sophus::SE2d{Sophus::SO2d{-Sophus::Constants<double>::pi() / 2.0}, Eigen::Vector2d{1.0, -1.0}};
}

template <>
Sophus::SE3d get_robot_pose_in_world<Sophus::SE3d>() {
  return Sophus::SE3d{Sophus::SO3d::rotZ(-Sophus::Constants<double>::pi() / 2.0), Eigen::Vector3d{1.0, -1.0, 0.0}};
}

template <typename T>
struct BearingSensorModelTests : public ::testing::Test {};

using BearingSensorModelTestsTypes = ::testing::Types<Sensor2D, Sensor3D>;

TYPED_TEST_SUITE(BearingSensorModelTests, BearingSensorModelTestsTypes);

TYPED_TEST(BearingSensorModelTests, SmokeTest) {
  auto sensor_model =
      TypeParam{get_default_model_params(), LandmarkMap(default_map_boundaries, {{{1.0, -1.0, 1.0}, 0}})};
}

TYPED_TEST(BearingSensorModelTests, BullsEyeDetection) {
  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  // test case where the landmark is exactly where we expected it
  auto map = LandmarkMap(default_map_boundaries, {{{1.0, -2.0, 1.0}, 0}});
  auto sensor_model = TypeParam{get_default_model_params(), std::move(map)};
  auto state_weighting_function = sensor_model({{{1.0, 0.0, 0.0}, 0}});
  EXPECT_NEAR(expected_aggregate_probability({1.0}), state_weighting_function(pose), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, MapUpdate) {
  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  // test calls to the update_map function
  auto map_1 = LandmarkMap(default_map_boundaries, {});
  auto map_2 = LandmarkMap(default_map_boundaries, {{{1.0, -2.0, 1.0}, 0}});
  auto sensor_model = TypeParam{get_default_model_params(), std::move(map_1)};
  auto state_weighting_function = sensor_model({{{1.0, 0.0, 0.0}, 0}});
  EXPECT_NEAR(expected_aggregate_probability({0.0}), state_weighting_function(pose), 1e-02);
  ASSERT_NO_THROW(sensor_model.update_map(std::move(map_2)));
  EXPECT_NEAR(expected_aggregate_probability({1.0}), state_weighting_function(pose), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, MultipleBullsEyeDetections) {
  // Test multiple detections of with different ids, all perfectly matching
  const auto sensor_model = TypeParam{
      get_default_model_params(),  //
      LandmarkMap(                 //
          default_map_boundaries,  //
          {
              {{1.0, -2.0, 0.0}, 0},  // landmark 0
              {{1.0, -2.0, 1.0}, 1},  // landmark 1
              {{1.0, -2.0, 2.0}, 2},  // landmark 2
          })};

  const auto state_weighting_function = sensor_model({
      {{+1.0, +0.0, -1.0}, 0},  // landmark 0 detection
      {{+1.0, +0.0, +0.0}, 1},  // landmark 1 detection
      {{+1.0, +0.0, +1.0}, 2},  // landmark 2 detection
  });

  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  EXPECT_NEAR(expected_aggregate_probability({1.0, 1.0, 1.0}), state_weighting_function(pose), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, OneStdInBearing) {
  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  // test case where the landmark is 1 std offset from the expected bearing
  auto map = LandmarkMap(default_map_boundaries, {{{1.0, -2.0, 1.0}, 0}});
  auto sensor_model = TypeParam{get_default_model_params(), std::move(map)};
  // baseline
  {
    auto state_weighting_function = sensor_model({{{1.0, 0.0, 0.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({1.0}), state_weighting_function(pose), 1e-02);
  }
  // 1 std left
  {
    auto state_weighting_function = sensor_model({{{1.0, 1.0, 0.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({0.6}), state_weighting_function(pose), 1e-02);
  }
  // 1 std right
  {
    auto state_weighting_function = sensor_model({{{1.0, -1.0, 0.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({0.6}), state_weighting_function(pose), 1e-02);
  }
  // 1 std up
  {
    auto state_weighting_function = sensor_model({{{1.0, 0.0, 1.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({0.6}), state_weighting_function(pose), 1e-02);
  }
  // 1 std down
  {
    auto state_weighting_function = sensor_model({{{1.0, 0.0, -1.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({0.6}), state_weighting_function(pose), 1e-02);
  }
}

TYPED_TEST(BearingSensorModelTests, NoSuchLandmark) {
  // perfect bearing measurement
  auto map = LandmarkMap(default_map_boundaries, {{{1.0, -1.0, 1.0}, 0}});
  const auto sensor_model = TypeParam{get_default_model_params(), std::move(map)};
  const auto state_weighting_function = sensor_model({{{1.0, 0.0, 0.0}, 99}});
  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  EXPECT_NEAR(expected_aggregate_probability({0.0}), state_weighting_function(pose), 1e-02);
}

}  // namespace beluga

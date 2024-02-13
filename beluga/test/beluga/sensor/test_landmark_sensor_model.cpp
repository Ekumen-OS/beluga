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

// external
#include <ciabatta/ciabatta.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

// standard library
#include <algorithm>
#include <cstdint>
#include <vector>

// project
#include <beluga/sensor.hpp>
#include <beluga/sensor/data/landmark_map.hpp>

namespace beluga {

namespace {

using Sensor2D = beluga::LandmarkSensorModel2d<LandmarkMap>;
using Sensor3D = beluga::LandmarkSensorModel3d<LandmarkMap>;

double expected_aggregate_probability(std::vector<double> landmark_probs) {
  // nav2_amcl formula, $1.0 + \sum_{i=1}^n p_i^3$
  return std::transform_reduce(
      landmark_probs.cbegin(), landmark_probs.cend(), 1.0, std::plus{}, [](const double v) { return v * v * v; });
}

LandmarkModelParam get_default_model_params() {
  LandmarkModelParam ret;
  ret.sigma_range = 1.0;
  ret.sigma_bearing = Sophus::Constants<double>::pi() / 2.0;  // 90 degrees
  return ret;
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

LandmarkMapBoundaries default_map_boundaries{-10.0, -10.0, 10.0, 10.0, 0.0, 0.0};

template <typename T>
struct LandmarkSensorModelTests : public ::testing::Test {};

using LandmarkSensorModelTestsTypes = ::testing::Types<Sensor2D, Sensor3D>;

TYPED_TEST_SUITE(LandmarkSensorModelTests, LandmarkSensorModelTestsTypes);

TYPED_TEST(LandmarkSensorModelTests, BullsEyeDetection) {
  // test case where the landmark is exactly where we expected it
  auto map = LandmarkMap{default_map_boundaries, {{{3.0, -2.0, 0.0}, 0}}};
  const auto sensor_model = TypeParam{get_default_model_params(), std::move(map)};
  const auto state_weighting_function = sensor_model({{{1.0, 2.0, 0.0}, 0}});
  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  EXPECT_NEAR(expected_aggregate_probability({1.0}), state_weighting_function(pose), 1e-02);
}

TYPED_TEST(LandmarkSensorModelTests, MapUpdate) {
  // test calls to the update_map function
  auto map_1 = LandmarkMap(default_map_boundaries, {});
  auto map_2 = LandmarkMap(default_map_boundaries, {{{3.0, -2.0, 0.0}, 0}});
  auto sensor_model = TypeParam{get_default_model_params(), std::move(map_1)};
  const auto state_weighting_function = sensor_model({{{1.0, 2.0, 0.0}, 0}});
  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  EXPECT_NEAR(expected_aggregate_probability({0.0}), state_weighting_function(pose), 1e-02);
  ASSERT_NO_THROW(sensor_model.update_map(std::move(map_2)));
  EXPECT_NEAR(expected_aggregate_probability({1.0}), state_weighting_function(pose), 1e-02);
}

TYPED_TEST(LandmarkSensorModelTests, MultipleBullsEyeDetections) {
  // Test multiple detections of with different ids, all perfectly matching
  const auto sensor_model = TypeParam{
      get_default_model_params(),  //
      LandmarkMap(                 //
          default_map_boundaries,  //
          {
              {{1.0, -2.0, 0.0}, 0},  // landmark 0
              {{1.0, -0.0, 0.0}, 1},  // landmark 1
              {{2.0, -1.0, 0.0}, 2},  // landmark 2
              {{0.0, -1.0, 0.0}, 3},  // landmark 3
          })};

  const auto state_weighting_function = sensor_model({
      {{+1.0, +0.0, 0.0}, 0},  // landmark 0 detection
      {{-1.0, +0.0, 0.0}, 1},  // landmark 1 detection
      {{+0.0, +1.0, 0.0}, 2},  // landmark 2 detection
      {{+0.0, -1.0, 0.0}, 3},  // landmark 3 detection
  });

  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  EXPECT_NEAR(expected_aggregate_probability({1.0, 1.0, 1.0, 1.0}), state_weighting_function(pose), 1e-02);
}

TYPED_TEST(LandmarkSensorModelTests, OneStdInRange) {
  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  // test case where the landmark is 1 std offset from the expected range
  auto map = LandmarkMap(default_map_boundaries, {{{1.0, -11.0, 0.0}, 0}});
  const auto sensor_model = TypeParam{get_default_model_params(), std::move(map)};
  // baseline
  {
    auto state_weighting_function = sensor_model({{{10.0, 0.0, 0.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({1.0}), state_weighting_function(pose), 1e-02);
  }
  // 1 std default
  {
    auto state_weighting_function = sensor_model({{{9.0, 0.0, 0.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({0.6}), state_weighting_function(pose), 1e-02);
  }
  // 1 std excess
  {
    auto state_weighting_function = sensor_model({{{11.0, 0.0, 0.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({0.6}), state_weighting_function(pose), 1e-02);
  }
}

TYPED_TEST(LandmarkSensorModelTests, OneStdInBearing) {
  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  // test case where the landmark is 1 std offset from the expected bearing
  auto map = LandmarkMap(default_map_boundaries, {{{1.0, -11.0, 0.0}, 0}});
  auto sensor_model = TypeParam{get_default_model_params(), std::move(map)};
  // baseline
  {
    auto state_weighting_function = sensor_model({{{10.0, 0.0, 0.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({1.0}), state_weighting_function(pose), 1e-02);
  }
  // 1 std default
  {
    auto state_weighting_function = sensor_model({{{0.0, 10.0, 0.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({0.6}), state_weighting_function(pose), 1e-02);
  }
  // 1 std excess
  {
    auto state_weighting_function = sensor_model({{{0.0, -10.0, 0.0}, 0}});
    EXPECT_NEAR(expected_aggregate_probability({0.6}), state_weighting_function(pose), 1e-02);
  }
}

TYPED_TEST(LandmarkSensorModelTests, NoSuchLandmark) {
  const auto pose = get_robot_pose_in_world<typename TypeParam::state_type>();
  // test case where there is not landmark in the map of the detected id
  auto map = LandmarkMap(default_map_boundaries, {{{0.0, 1.0, 0.0}, 99}});
  auto sensor_model = TypeParam{get_default_model_params(), std::move(map)};
  auto state_weighting_function = sensor_model({{{0.0, 2.0, 0.0}, 88}});
  EXPECT_NEAR(expected_aggregate_probability({0.0}), state_weighting_function(pose), 1e-02);
}

}  // namespace

}  // namespace beluga

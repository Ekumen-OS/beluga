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

// standard library
#include <algorithm>
#include <random>
#include <vector>

// project
#include <beluga/sensor.hpp>
#include <beluga/sensor/data/landmark_map.hpp>

namespace beluga {

using Sensor2D = beluga::BearingSensorModel2d<LandmarkMap>;
using Sensor3D = beluga::BearingSensorModel3d<LandmarkMap>;

LandmarkMapBoundaries default_map_boundaries{-10.0, -10.0, 10.0, 10.0, 0.0, 0.0};

double expected_aggregate_probability(std::vector<double> landmark_probs) {
  // nav2_amcl formula, $1.0 + \sum_{i=1}^n p_i^3$
  return std::transform_reduce(
      landmark_probs.cbegin(), landmark_probs.cend(), 1.0, std::plus{}, [](const double v) { return v * v * v; });
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
  auto uut = std::make_unique<TypeParam>(
      get_default_model_params(), LandmarkMap(default_map_boundaries, {{{1.0, -1.0, 1.0}, 0}}));
}

TYPED_TEST(BearingSensorModelTests, BullsEyeDetection) {
  // test case where the landmark is exactly where we expected it
  auto uut = std::make_unique<TypeParam>(
      get_default_model_params(), LandmarkMap(default_map_boundaries, {{{1.0, -2.0, 1.0}, 0}}));
  ASSERT_NO_THROW(uut->update_sensor({{{1.0, 0.0, 0.0}, 0}}));
  EXPECT_NEAR(
      expected_aggregate_probability({1.0}),
      uut->importance_weight(get_robot_pose_in_world<typename TypeParam::state_type>()), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, MapUpdate) {
  // test calls to the update_map function
  auto map_1 = LandmarkMap(default_map_boundaries, {});
  auto map_2 = LandmarkMap(default_map_boundaries, {{{1.0, -2.0, 1.0}, 0}});
  auto uut = std::make_unique<TypeParam>(get_default_model_params(), std::move(map_1));
  ASSERT_NO_THROW(uut->update_sensor({{{1.0, 0.0, 0.0}, 0}}));
  EXPECT_NEAR(
      expected_aggregate_probability({0.0}),
      uut->importance_weight(get_robot_pose_in_world<typename TypeParam::state_type>()), 1e-02);
  ASSERT_NO_THROW(uut->update_map(std::move(map_2)));
  EXPECT_NEAR(
      expected_aggregate_probability({1.0}),
      uut->importance_weight(get_robot_pose_in_world<typename TypeParam::state_type>()), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, MultipleBullsEyeDetections) {
  // Test multiple detections of with different ids, all perfectly matching
  auto uut = std::make_unique<TypeParam>(
      get_default_model_params(),  //
      LandmarkMap(                 //
          default_map_boundaries,  //
          {
              {{1.0, -2.0, 0.0}, 0},  // landmark 0
              {{1.0, -2.0, 1.0}, 1},  // landmark 1
              {{1.0, -2.0, 2.0}, 2},  // landmark 2
          }));

  ASSERT_NO_THROW(uut->update_sensor({
      {{+1.0, +0.0, -1.0}, 0},  // landmark 0 detection
      {{+1.0, +0.0, +0.0}, 1},  // landmark 1 detection
      {{+1.0, +0.0, +1.0}, 2},  // landmark 2 detection
  }));

  EXPECT_NEAR(
      expected_aggregate_probability({1.0, 1.0, 1.0}),
      uut->importance_weight(get_robot_pose_in_world<typename TypeParam::state_type>()), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, OneStdInBearing) {
  // test case where the landmark is 1 std offset from the expected bearing
  auto uut = std::make_unique<TypeParam>(
      get_default_model_params(), LandmarkMap(default_map_boundaries, {{{1.0, -2.0, 1.0}, 0}}));
  // baseline
  ASSERT_NO_THROW(uut->update_sensor({{{1.0, 0.0, 0.0}, 0}}));
  EXPECT_NEAR(
      expected_aggregate_probability({1.0}),
      uut->importance_weight(get_robot_pose_in_world<typename TypeParam::state_type>()), 1e-02);
  // 1 std left
  ASSERT_NO_THROW(uut->update_sensor({{{1.0, 1.0, 0.0}, 0}}));
  EXPECT_NEAR(
      expected_aggregate_probability({0.6}),
      uut->importance_weight(get_robot_pose_in_world<typename TypeParam::state_type>()), 1e-02);
  // 1 std right
  ASSERT_NO_THROW(uut->update_sensor({{{1.0, -1.0, 0.0}, 0}}));
  EXPECT_NEAR(
      expected_aggregate_probability({0.6}),
      uut->importance_weight(get_robot_pose_in_world<typename TypeParam::state_type>()), 1e-02);
  // 1 std up
  ASSERT_NO_THROW(uut->update_sensor({{{1.0, 0.0, 1.0}, 0}}));
  EXPECT_NEAR(
      expected_aggregate_probability({0.6}),
      uut->importance_weight(get_robot_pose_in_world<typename TypeParam::state_type>()), 1e-02);
  // 1 std down
  ASSERT_NO_THROW(uut->update_sensor({{{1.0, 0.0, -1.0}, 0}}));
  EXPECT_NEAR(
      expected_aggregate_probability({0.6}),
      uut->importance_weight(get_robot_pose_in_world<typename TypeParam::state_type>()), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, NoSuchLandmark) {
  // perfect bearing measurement
  auto uut = std::make_unique<TypeParam>(
      get_default_model_params(), LandmarkMap(default_map_boundaries, {{{1.0, -1.0, 1.0}, 0}}));
  ASSERT_NO_THROW(uut->update_sensor({{{1.0, 0.0, 0.0}, 99}}));
  EXPECT_NEAR(
      expected_aggregate_probability({0.0}),
      uut->importance_weight(get_robot_pose_in_world<typename TypeParam::state_type>()), 1e-02);
}

}  // namespace beluga

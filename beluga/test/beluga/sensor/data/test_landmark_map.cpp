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
#include <optional>

// external
#include <sophus/se3.hpp>

// project
#include "beluga/sensor/data/landmark_map.hpp"
#include "beluga/types/landmark_detection_types.hpp"

namespace {

struct LandmarkMapCartesianTest : public ::testing::Test {
  beluga::LandmarkMapBoundaries default_map_boundaries{
      Eigen::Vector3d{0.0, 1.0, 2.0}, Eigen::Vector3d{10.0, 11.0, 12.0}};
};

TEST_F(LandmarkMapCartesianTest, SmokeTest) {
  ASSERT_NO_THROW(beluga::LandmarkMap(default_map_boundaries, beluga::LandmarkMap::landmarks_set_position_data{}));
}

TEST_F(LandmarkMapCartesianTest, SimpleMapLoading) {
  const auto landmark_10 = beluga::LandmarkPosition3{+1.0, +2.0, +3.0};
  const auto landmark_11 = beluga::LandmarkPosition3{+1.0, +2.0, +3.0};
  const auto landmark_20 = beluga::LandmarkPosition3{+5.0, +6.0, +7.0};

  auto uut = beluga::LandmarkMap(default_map_boundaries, {{landmark_10, 0}, {landmark_11, 0}, {landmark_20, 1}});

  {
    const auto nearest = uut.find_nearest_landmark({1.0, 2.0, 3.0}, 0);
    ASSERT_TRUE(nearest.has_value());
    EXPECT_NEAR(0.0, (landmark_10 - *nearest).norm(), 1e-6);
  }

  {
    const auto nearest = uut.find_nearest_landmark({1.0, 2.0, 3.0}, 1);
    ASSERT_TRUE(nearest.has_value());
    EXPECT_NEAR(0.0, (landmark_20 - *nearest).norm(), 1e-6);
  }

  {
    const auto nearest = uut.find_nearest_landmark({1.0, 2.0, 3.0}, 99);
    ASSERT_FALSE(nearest.has_value());
  }
}

TEST_F(LandmarkMapCartesianTest, EmptyMap) {
  auto uut = beluga::LandmarkMap(default_map_boundaries, beluga::LandmarkMap::landmarks_set_position_data{});
  const auto nearest = uut.find_nearest_landmark({1.0, 2.0, 3.0}, 0);
  ASSERT_FALSE(nearest.has_value());
}

struct LandmarkMapBearingTest : public ::testing::Test {
  beluga::LandmarkMapBoundaries default_map_boundaries{
      Eigen::Vector3d{0.0, 1.0, 2.0}, Eigen::Vector3d{10.0, 11.0, 12.0}};

  beluga::LandmarkMap uut{
      default_map_boundaries,
      {
          // category 0
          {{+9.0, +0.0, +1.0}, 0},
          {{+0.0, +9.0, +1.0}, 0},
          {{+0.0, +0.0, +9.0}, 0},
          // category 1
          {{-9.0, +0.0, +1.0}, 1},
          {{+0.0, -9.0, +1.0}, 1},
          {{+0.0, +0.0, -9.0}, 1},
          // category 2
          {{+0.0, +0.0, -9.0}, 2},
      }};

  Sophus::SE3d sensor_pose_in_world{Sophus::SO3d{}, Eigen::Vector3d{0.0, 0.0, 1.0}};
};

TEST_F(LandmarkMapBearingTest, MapLimits) {
  ASSERT_DOUBLE_EQ(uut.map_limits().min().x(), 0.0);
  ASSERT_DOUBLE_EQ(uut.map_limits().max().x(), 10.0);
  ASSERT_DOUBLE_EQ(uut.map_limits().min().y(), 1.0);
  ASSERT_DOUBLE_EQ(uut.map_limits().max().y(), 11.0);
  ASSERT_DOUBLE_EQ(uut.map_limits().min().z(), 2.0);
  ASSERT_DOUBLE_EQ(uut.map_limits().max().z(), 12.0);
}

TEST_F(LandmarkMapBearingTest, TrivialQuery1) {
  const auto expected_bearing = beluga::LandmarkBearing3{1.0, 0.0, 0.0};
  const auto nearest = uut.find_closest_bearing_landmark(expected_bearing, 0, sensor_pose_in_world);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_NEAR(0.0, (*nearest - expected_bearing).norm(), 1e-6);
}

TEST_F(LandmarkMapBearingTest, TrivialQuery2) {
  const auto expected_bearing = beluga::LandmarkBearing3{-1.0, 0.0, 0.0};
  const auto nearest = uut.find_closest_bearing_landmark(expected_bearing, 1, sensor_pose_in_world);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_NEAR(0.0, (*nearest - expected_bearing).norm(), 1e-6);
}

TEST_F(LandmarkMapBearingTest, FeatureInTotallyDifferentDirection) {
  const auto detection_bearing = beluga::LandmarkBearing3{1.0, 0.0, 0.0};
  const auto expected_bearing = beluga::LandmarkBearing3{0.0, 0.0, -1.0};
  const auto nearest = uut.find_closest_bearing_landmark(detection_bearing, 2, sensor_pose_in_world);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_NEAR(0.0, (*nearest - expected_bearing).norm(), 1e-6);
}

TEST_F(LandmarkMapBearingTest, NoSuchFeature) {
  const auto nearest =
      uut.find_closest_bearing_landmark(beluga::LandmarkBearing3{1.0, 0.0, 0.0}, 99, sensor_pose_in_world);
  ASSERT_FALSE(nearest.has_value());
}

}  // namespace

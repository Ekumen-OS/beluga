// Copyright 2026 Ekumen, Inc.
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

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/mocks/mock_transform.hpp>

namespace {

using testing::_;
using testing::Return;

constexpr double kTolerance = 1e-6;

TEST(TransformInterfaceTest, LookupTransformSE2) {
  beluga_estimation::MockTransform transform_interface;
  beluga_estimation::TimePoint time;
  beluga_estimation::SE3d expected_se3{
      Eigen::Quaterniond{Eigen::AngleAxisd{M_PI / 3, Eigen::Vector3d::UnitZ()}}, Eigen::Vector3d{1.0, 2.0, 3.0}};

  EXPECT_CALL(transform_interface, LookupTransform("map", "odom", time, _)).WillOnce(Return(expected_se3));

  const auto result = transform_interface.LookupTransformSE2("map", "odom", time);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->translation().x(), 1.0, kTolerance);
  EXPECT_NEAR(result->translation().y(), 2.0, kTolerance);
  EXPECT_NEAR(result->so2().log(), M_PI / 3, kTolerance);
}

TEST(TransformInterfaceTest, LookupTransformSO3) {
  beluga_estimation::MockTransform transform_interface;
  beluga_estimation::TimePoint time;
  beluga_estimation::SE3d expected_se3{
      Eigen::Quaterniond{Eigen::AngleAxisd{M_PI / 4, Eigen::Vector3d::UnitX()}}, Eigen::Vector3d{1.0, 2.0, 3.0}};

  EXPECT_CALL(transform_interface, LookupTransform("map", "odom", time, _)).WillOnce(Return(expected_se3));

  const auto result = transform_interface.LookupTransformSO3("map", "odom", time);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->log().x(), M_PI / 4, kTolerance);
  EXPECT_NEAR(result->log().y(), 0.0, kTolerance);
  EXPECT_NEAR(result->log().z(), 0.0, kTolerance);
}

TEST(TransformInterfaceTest, LookupTransformSO2) {
  beluga_estimation::MockTransform transform_interface;
  beluga_estimation::TimePoint time;
  beluga_estimation::SE3d expected_se3{
      Eigen::Quaterniond{Eigen::AngleAxisd{-M_PI / 6, Eigen::Vector3d::UnitZ()}}, Eigen::Vector3d{1.0, 2.0, 3.0}};

  EXPECT_CALL(transform_interface, LookupTransform("map", "odom", time, _)).WillOnce(Return(expected_se3));

  const auto result = transform_interface.LookupTransformSO2("map", "odom", time);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->log(), -M_PI / 6, kTolerance);
}

TEST(TransformInterfaceTest, LookupTransformNullopt) {
  beluga_estimation::MockTransform transform_interface;
  beluga_estimation::TimePoint time;

  EXPECT_CALL(transform_interface, LookupTransform("map", "odom", time, _)).WillRepeatedly(Return(std::nullopt));

  EXPECT_FALSE(transform_interface.LookupTransformSE2("map", "odom", time).has_value());
  EXPECT_FALSE(transform_interface.LookupTransformSO3("map", "odom", time).has_value());
  EXPECT_FALSE(transform_interface.LookupTransformSO2("map", "odom", time).has_value());
}

TEST(TransformInterfaceTest, BroadcastTransformSE2) {
  beluga_estimation::MockTransform transform_interface;
  beluga_estimation::TimePoint time;
  beluga_estimation::SE2d se2_transform{M_PI / 4, Eigen::Vector2d{3.0, 4.0}};

  EXPECT_CALL(transform_interface, BroadcastTransform("map", "odom", time, _))
      .WillOnce(
          [](std::string_view, std::string_view, beluga_estimation::TimePoint, const beluga_estimation::SE3d& se3) {
            EXPECT_NEAR(se3.translation().x(), 3.0, kTolerance);
            EXPECT_NEAR(se3.translation().y(), 4.0, kTolerance);
            EXPECT_NEAR(se3.translation().z(), 0.0, kTolerance);
            EXPECT_NEAR(se3.so3().log().z(), M_PI / 4, kTolerance);
            EXPECT_NEAR(se3.so3().log().x(), 0.0, kTolerance);
            EXPECT_NEAR(se3.so3().log().y(), 0.0, kTolerance);
          });

  transform_interface.BroadcastTransform("map", "odom", time, se2_transform);
}

TEST(TransformInterfaceTest, BroadcastTransformSO3) {
  beluga_estimation::MockTransform transform_interface;
  beluga_estimation::TimePoint time;
  beluga_estimation::SO3d so3_transform{Eigen::Quaterniond{Eigen::AngleAxisd{M_PI / 2, Eigen::Vector3d::UnitY()}}};

  EXPECT_CALL(transform_interface, BroadcastTransform("map", "odom", time, _))
      .WillOnce(
          [](std::string_view, std::string_view, beluga_estimation::TimePoint, const beluga_estimation::SE3d& se3) {
            EXPECT_NEAR(se3.translation().x(), 0.0, kTolerance);
            EXPECT_NEAR(se3.translation().y(), 0.0, kTolerance);
            EXPECT_NEAR(se3.translation().z(), 0.0, kTolerance);
            EXPECT_NEAR(se3.so3().log().x(), 0.0, kTolerance);
            EXPECT_NEAR(se3.so3().log().y(), M_PI / 2, kTolerance);
            EXPECT_NEAR(se3.so3().log().z(), 0.0, kTolerance);
          });

  transform_interface.BroadcastTransform("map", "odom", time, so3_transform);
}

}  // namespace

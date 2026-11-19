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

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/algebra/interpolation.hpp>
#include <beluga_estimation/common/algebra/projection.hpp>
#include <beluga_estimation/common/views.hpp>

namespace {

// ============================================================================
// SO2 Tests
// ============================================================================

TEST(Refs, SO2RefFromSO2) {
  beluga_estimation::SO2d so2(M_PI / 4.0);
  beluga_estimation::Ref<beluga_estimation::SO2d> ref(so2);

  // Verify ref acts like so2
  EXPECT_NEAR(ref.log(), M_PI / 4.0, 1e-10);
  EXPECT_NEAR(ref.unit_complex().x(), std::cos(M_PI / 4.0), 1e-10);
  EXPECT_NEAR(ref.unit_complex().y(), std::sin(M_PI / 4.0), 1e-10);
}

TEST(Refs, SO2RefFromMap) {
  std::vector<double> data = {std::cos(M_PI / 3.0), std::sin(M_PI / 3.0)};
  beluga_estimation::Map<beluga_estimation::SO2d> map(data.data());
  beluga_estimation::Ref<beluga_estimation::SO2d> ref(map);

  // Verify ref works
  EXPECT_NEAR(ref.log(), M_PI / 3.0, 1e-10);
  EXPECT_NEAR(ref.unit_complex().x(), std::cos(M_PI / 3.0), 1e-10);
}

TEST(Refs, SO2RefModifiesOriginal) {
  beluga_estimation::SO2d so2(0.0);
  beluga_estimation::Ref<beluga_estimation::SO2d> ref(so2);

  // Modify through ref
  ref = beluga_estimation::SO2d(M_PI / 2.0);

  // Verify original is modified
  EXPECT_NEAR(so2.log(), M_PI / 2.0, 1e-10);
}

TEST(Refs, ConstSO2Ref) {
  const beluga_estimation::SO2d so2(M_PI / 6.0);
  beluga_estimation::Ref<const beluga_estimation::SO2d> ref(so2);

  // Verify const-correctness
  EXPECT_NEAR(ref.log(), M_PI / 6.0, 1e-10);
  EXPECT_NEAR(ref.unit_complex().x(), std::cos(M_PI / 6.0), 1e-10);
}

TEST(Refs, ConstSO2RefFromMap) {
  std::vector<double> data = {std::cos(M_PI / 8.0), std::sin(M_PI / 8.0)};
  const beluga_estimation::Map<const beluga_estimation::SO2d> map(data.data());
  beluga_estimation::Ref<const beluga_estimation::SO2d> ref(map);

  EXPECT_NEAR(ref.log(), M_PI / 8.0, 1e-10);
}

TEST(Refs, SO2RefOperations) {
  beluga_estimation::SO2d so2_a(M_PI / 4.0);
  beluga_estimation::SO2d so2_b(M_PI / 4.0);
  beluga_estimation::Ref<beluga_estimation::SO2d> ref_a(so2_a);
  beluga_estimation::Ref<beluga_estimation::SO2d> ref_b(so2_b);

  // Test multiplication
  auto result = ref_a * ref_b;
  EXPECT_NEAR(result.log(), M_PI / 2.0, 1e-10);

  // Test inverse
  auto inv = ref_a.inverse();
  EXPECT_NEAR(inv.log(), -M_PI / 4.0, 1e-10);
}

// ============================================================================
// SO3 Tests
// ============================================================================

TEST(Refs, SO3RefFromSO3) {
  beluga_estimation::SO3d so3 = beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(M_PI / 4.0, 0.0, 0.0));
  beluga_estimation::Ref<beluga_estimation::SO3d> ref(so3);

  // Verify ref acts like so3
  auto log_ref = ref.log();
  auto log_orig = so3.log();
  EXPECT_NEAR(log_ref.x(), log_orig.x(), 1e-10);
  EXPECT_NEAR(log_ref.y(), log_orig.y(), 1e-10);
  EXPECT_NEAR(log_ref.z(), log_orig.z(), 1e-10);
}

TEST(Refs, SO3RefFromMap) {
  beluga_estimation::SO3d so3 = beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(0.0, M_PI / 3.0, 0.0));
  std::vector<double> data(4);
  std::copy_n(so3.data(), 4, data.begin());

  beluga_estimation::Map<beluga_estimation::SO3d> map(data.data());
  beluga_estimation::Ref<beluga_estimation::SO3d> ref(map);

  auto log_ref = ref.log();
  EXPECT_NEAR(log_ref.y(), M_PI / 3.0, 1e-10);
}

TEST(Refs, SO3RefModifiesOriginal) {
  beluga_estimation::SO3d so3 = beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(0.1, 0.2, 0.3));
  beluga_estimation::Ref<beluga_estimation::SO3d> ref(so3);

  // Modify through ref
  beluga_estimation::SO3d new_value = beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(1.0, 0.0, 0.0));
  ref = new_value;

  // Verify original is modified
  auto log_orig = so3.log();
  EXPECT_NEAR(log_orig.x(), 1.0, 1e-10);
  EXPECT_NEAR(log_orig.y(), 0.0, 1e-10);
}

TEST(Refs, ConstSO3Ref) {
  const beluga_estimation::SO3d so3 = beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(0.5, 0.5, 0.5));
  beluga_estimation::Ref<const beluga_estimation::SO3d> ref(so3);

  auto log_ref = ref.log();
  auto log_orig = so3.log();
  EXPECT_NEAR(log_ref.norm(), log_orig.norm(), 1e-10);
}

TEST(Refs, SO3RefOperations) {
  beluga_estimation::SO3d so3_a = beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(M_PI / 4.0, 0.0, 0.0));
  beluga_estimation::SO3d so3_b = beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(0.0, M_PI / 4.0, 0.0));
  beluga_estimation::Ref<beluga_estimation::SO3d> ref_a(so3_a);
  beluga_estimation::Ref<beluga_estimation::SO3d> ref_b(so3_b);

  // Test multiplication
  auto result = ref_a * ref_b;
  auto log_result = result.log();
  EXPECT_GT(log_result.norm(), 0.0);  // Non-trivial result

  // Test inverse
  auto inv = ref_a.inverse();
  auto identity_check = ref_a * inv;
  EXPECT_NEAR(identity_check.log().norm(), 0.0, 1e-10);
}

// ============================================================================
// SE2 Tests
// ============================================================================

TEST(Refs, SE2RefFromSE2) {
  beluga_estimation::SE2d se2(beluga_estimation::SO2d(M_PI / 4.0), beluga_estimation::Vector2d(1.0, 2.0));
  beluga_estimation::Ref<beluga_estimation::SE2d> ref(se2);

  // Verify ref acts like se2
  EXPECT_NEAR(ref.so2().log(), M_PI / 4.0, 1e-10);
  EXPECT_NEAR(ref.translation().x(), 1.0, 1e-10);
  EXPECT_NEAR(ref.translation().y(), 2.0, 1e-10);
}

TEST(Refs, SE2RefFromMap) {
  std::vector<double> data = {std::cos(M_PI / 3.0), std::sin(M_PI / 3.0), 3.0, 4.0};
  beluga_estimation::Map<beluga_estimation::SE2d> map(data.data());
  beluga_estimation::Ref<beluga_estimation::SE2d> ref(map);

  EXPECT_NEAR(ref.so2().log(), M_PI / 3.0, 1e-10);
  EXPECT_NEAR(ref.translation().x(), 3.0, 1e-10);
  EXPECT_NEAR(ref.translation().y(), 4.0, 1e-10);
}

TEST(Refs, SE2RefModifiesOriginal) {
  beluga_estimation::SE2d se2(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(0.0, 0.0));
  beluga_estimation::Ref<beluga_estimation::SE2d> ref(se2);

  // Modify through ref
  ref = beluga_estimation::SE2d(beluga_estimation::SO2d(M_PI / 2.0), beluga_estimation::Vector2d(5.0, 6.0));

  // Verify original is modified
  EXPECT_NEAR(se2.so2().log(), M_PI / 2.0, 1e-10);
  EXPECT_NEAR(se2.translation().x(), 5.0, 1e-10);
  EXPECT_NEAR(se2.translation().y(), 6.0, 1e-10);
}

TEST(Refs, SE2RefModifiesComponents) {
  beluga_estimation::SE2d se2(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(1.0, 2.0));
  beluga_estimation::Ref<beluga_estimation::SE2d> ref(se2);

  // Modify components through ref
  ref.so2() = beluga_estimation::SO2d(M_PI);
  ref.translation().x() = 10.0;
  ref.translation().y() = 20.0;

  // Verify original is modified
  EXPECT_NEAR(se2.so2().log(), M_PI, 1e-10);
  EXPECT_NEAR(se2.translation().x(), 10.0, 1e-10);
  EXPECT_NEAR(se2.translation().y(), 20.0, 1e-10);
}

TEST(Refs, ConstSE2Ref) {
  const beluga_estimation::SE2d se2(beluga_estimation::SO2d(M_PI / 6.0), beluga_estimation::Vector2d(7.0, 8.0));
  beluga_estimation::Ref<const beluga_estimation::SE2d> ref(se2);

  EXPECT_NEAR(ref.so2().log(), M_PI / 6.0, 1e-10);
  EXPECT_NEAR(ref.translation().x(), 7.0, 1e-10);
  EXPECT_NEAR(ref.translation().y(), 8.0, 1e-10);
}

TEST(Refs, ConstSE2RefFromMap) {
  std::vector<double> data = {std::cos(M_PI / 8.0), std::sin(M_PI / 8.0), 9.0, 10.0};
  const beluga_estimation::Map<const beluga_estimation::SE2d> map(data.data());
  beluga_estimation::Ref<const beluga_estimation::SE2d> ref(map);

  EXPECT_NEAR(ref.so2().log(), M_PI / 8.0, 1e-10);
  EXPECT_NEAR(ref.translation().x(), 9.0, 1e-10);
}

TEST(Refs, SE2RefOperations) {
  beluga_estimation::SE2d se2_a(beluga_estimation::SO2d(M_PI / 4.0), beluga_estimation::Vector2d(1.0, 0.0));
  beluga_estimation::SE2d se2_b(beluga_estimation::SO2d(M_PI / 4.0), beluga_estimation::Vector2d(0.0, 1.0));
  beluga_estimation::Ref<beluga_estimation::SE2d> ref_a(se2_a);
  beluga_estimation::Ref<beluga_estimation::SE2d> ref_b(se2_b);

  // Test multiplication
  auto result = ref_a * ref_b;
  EXPECT_NEAR(result.so2().log(), M_PI / 2.0, 1e-10);

  // Test inverse
  auto inv = ref_a.inverse();
  auto identity_check = ref_a * inv;
  EXPECT_NEAR(identity_check.so2().log(), 0.0, 1e-10);
  EXPECT_NEAR(identity_check.translation().norm(), 0.0, 1e-10);
}

// ============================================================================
// SE3 Tests
// ============================================================================

TEST(Refs, SE3RefFromSE3) {
  beluga_estimation::SE3d se3(
      beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(M_PI / 4.0, 0.0, 0.0)),
      beluga_estimation::Vector3d(1.0, 2.0, 3.0));
  beluga_estimation::Ref<beluga_estimation::SE3d> ref(se3);

  // Verify ref acts like se3
  auto log_so3 = ref.so3().log();
  EXPECT_NEAR(log_so3.x(), M_PI / 4.0, 1e-10);
  EXPECT_NEAR(ref.translation().x(), 1.0, 1e-10);
  EXPECT_NEAR(ref.translation().y(), 2.0, 1e-10);
  EXPECT_NEAR(ref.translation().z(), 3.0, 1e-10);
}

TEST(Refs, SE3RefFromMap) {
  beluga_estimation::SE3d se3_orig(
      beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(0.0, M_PI / 3.0, 0.0)),
      beluga_estimation::Vector3d(4.0, 5.0, 6.0));
  std::vector<double> data(7);
  std::copy_n(se3_orig.data(), 7, data.begin());

  beluga_estimation::Map<beluga_estimation::SE3d> map(data.data());
  beluga_estimation::Ref<beluga_estimation::SE3d> ref(map);

  auto log_so3 = ref.so3().log();
  EXPECT_NEAR(log_so3.y(), M_PI / 3.0, 1e-10);
  EXPECT_NEAR(ref.translation().x(), 4.0, 1e-10);
}

TEST(Refs, SE3RefModifiesOriginal) {
  beluga_estimation::SE3d se3(
      beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(0.1, 0.2, 0.3)),
      beluga_estimation::Vector3d(1.0, 2.0, 3.0));
  beluga_estimation::Ref<beluga_estimation::SE3d> ref(se3);

  // Modify through ref
  beluga_estimation::SE3d new_value(
      beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(1.0, 0.0, 0.0)),
      beluga_estimation::Vector3d(7.0, 8.0, 9.0));
  ref = new_value;

  // Verify original is modified
  auto log_orig = se3.so3().log();
  EXPECT_NEAR(log_orig.x(), 1.0, 1e-10);
  EXPECT_NEAR(se3.translation().x(), 7.0, 1e-10);
}

TEST(Refs, SE3RefModifiesComponents) {
  beluga_estimation::SE3d se3(
      beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(0.0, 0.0, 0.0)),
      beluga_estimation::Vector3d(1.0, 2.0, 3.0));
  beluga_estimation::Ref<beluga_estimation::SE3d> ref(se3);

  // Modify components through ref
  ref.so3() = beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(M_PI / 2.0, 0.0, 0.0));
  ref.translation().x() = 10.0;
  ref.translation().y() = 20.0;
  ref.translation().z() = 30.0;

  // Verify original is modified
  auto log_orig = se3.so3().log();
  EXPECT_NEAR(log_orig.x(), M_PI / 2.0, 1e-10);
  EXPECT_NEAR(se3.translation().x(), 10.0, 1e-10);
  EXPECT_NEAR(se3.translation().y(), 20.0, 1e-10);
  EXPECT_NEAR(se3.translation().z(), 30.0, 1e-10);
}

TEST(Refs, ConstSE3Ref) {
  const beluga_estimation::SE3d se3(
      beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(M_PI / 6.0, 0.0, 0.0)),
      beluga_estimation::Vector3d(11.0, 12.0, 13.0));
  beluga_estimation::Ref<const beluga_estimation::SE3d> ref(se3);

  auto log_so3 = ref.so3().log();
  EXPECT_NEAR(log_so3.x(), M_PI / 6.0, 1e-10);
  EXPECT_NEAR(ref.translation().x(), 11.0, 1e-10);
}

TEST(Refs, ConstSE3RefFromMap) {
  beluga_estimation::SE3d se3_orig(
      beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(0.0, 0.0, M_PI / 8.0)),
      beluga_estimation::Vector3d(14.0, 15.0, 16.0));
  std::vector<double> data(7);
  std::copy_n(se3_orig.data(), 7, data.begin());

  const beluga_estimation::Map<const beluga_estimation::SE3d> map(data.data());
  beluga_estimation::Ref<const beluga_estimation::SE3d> ref(map);

  auto log_so3 = ref.so3().log();
  EXPECT_NEAR(log_so3.z(), M_PI / 8.0, 1e-10);
  EXPECT_NEAR(ref.translation().x(), 14.0, 1e-10);
}

TEST(Refs, SE3RefOperations) {
  beluga_estimation::SE3d se3_a(
      beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(M_PI / 4.0, 0.0, 0.0)),
      beluga_estimation::Vector3d(1.0, 0.0, 0.0));
  beluga_estimation::SE3d se3_b(
      beluga_estimation::SO3d::exp(beluga_estimation::Vector3d(0.0, M_PI / 4.0, 0.0)),
      beluga_estimation::Vector3d(0.0, 1.0, 0.0));
  beluga_estimation::Ref<beluga_estimation::SE3d> ref_a(se3_a);
  beluga_estimation::Ref<beluga_estimation::SE3d> ref_b(se3_b);

  // Test multiplication
  auto result = ref_a * ref_b;
  auto log_result = result.so3().log();
  EXPECT_GT(log_result.norm(), 0.0);  // Non-trivial result

  // Test inverse
  auto inv = ref_a.inverse();
  auto identity_check = ref_a * inv;
  EXPECT_NEAR(identity_check.so3().log().norm(), 0.0, 1e-10);
  EXPECT_NEAR(identity_check.translation().norm(), 0.0, 1e-10);
}

// ============================================================================
// Map / Ref Interoperability Tests
// ============================================================================

TEST(MapAndRef, MatrixRefFromMdspanMap) {
  beluga_estimation::MatrixXd matrix(3, 3);
  matrix << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  auto map =
      beluga_estimation::Map<beluga_estimation::MatrixXd>(beluga_estimation::to_view(matrix.topLeftCorner(2, 2)));
  beluga_estimation::Ref<beluga_estimation::MatrixXd> ref(map);

  EXPECT_EQ(ref.rows(), 2);
  EXPECT_EQ(ref.cols(), 2);
  EXPECT_DOUBLE_EQ(ref(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(ref(1, 1), 5.0);

  ref(1, 0) = 42.0;
  EXPECT_DOUBLE_EQ(matrix(1, 0), 42.0);
}

TEST(MapAndRef, ConstMatrixRefFromMdspanMap) {
  const beluga_estimation::MatrixXd matrix =
      (beluga_estimation::MatrixXd(2, 3) << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();

  auto map = beluga_estimation::Map<const beluga_estimation::MatrixXd>(beluga_estimation::to_view(matrix));
  beluga_estimation::Ref<const beluga_estimation::MatrixXd> ref(map);

  EXPECT_EQ(ref.rows(), 2);
  EXPECT_EQ(ref.cols(), 3);
  EXPECT_DOUBLE_EQ(ref(0, 2), 3.0);
  EXPECT_DOUBLE_EQ(ref(1, 1), 5.0);
}

TEST(MapAndRef, VectorMapFromSpan) {
  std::vector<double> values = {1.0, 2.0, 3.0};

  beluga_estimation::Map<beluga_estimation::Vector3d> mapped(beluga_estimation::to_view(values));
  mapped(1) = 42.0;

  EXPECT_DOUBLE_EQ(values[1], 42.0);
}

TEST(MapAndRef, DynamicVectorMapFromSpan) {
  std::vector<double> values = {4.0, 5.0, 6.0};

  beluga_estimation::Map<beluga_estimation::VectorXd> mapped(beluga_estimation::to_view(values));
  mapped(2) = -1.0;

  EXPECT_DOUBLE_EQ(values[2], -1.0);
}

TEST(MapAndRef, SophusSO2MapInteroperatesWithRef) {
  std::vector<double> data = {std::cos(M_PI / 4.0), std::sin(M_PI / 4.0)};

  beluga_estimation::Map<beluga_estimation::SO2d> map(data.data());
  beluga_estimation::Ref<beluga_estimation::SO2d> ref(map);

  EXPECT_NEAR(ref.log(), M_PI / 4.0, 1e-10);
  ref = beluga_estimation::SO2d(M_PI / 3.0);
  EXPECT_NEAR(map.log(), M_PI / 3.0, 1e-10);
}

TEST(MapAndRef, SophusConstSE2MapInteroperatesWithRef) {
  std::vector<double> data = {std::cos(M_PI / 6.0), std::sin(M_PI / 6.0), 2.0, 3.0};

  const beluga_estimation::Map<const beluga_estimation::SE2d> map(data.data());
  beluga_estimation::Ref<const beluga_estimation::SE2d> ref(map);

  EXPECT_NEAR(ref.so2().log(), M_PI / 6.0, 1e-10);
  EXPECT_DOUBLE_EQ(ref.translation().x(), 2.0);
  EXPECT_DOUBLE_EQ(ref.translation().y(), 3.0);
}

// ============================================================================
// Projection Tests
// ============================================================================

TEST(Projection, ExtractYawZero) {
  const beluga_estimation::Quaterniond q(Eigen::AngleAxisd(0.0, beluga_estimation::Vector3d::UnitZ()));
  EXPECT_NEAR(beluga_estimation::ExtractYaw(q), 0.0, 1e-9);
}

TEST(Projection, ExtractYawPiOver2) {
  const beluga_estimation::Quaterniond q(Eigen::AngleAxisd(M_PI / 2.0, beluga_estimation::Vector3d::UnitZ()));
  EXPECT_NEAR(beluga_estimation::ExtractYaw(q), M_PI / 2.0, 1e-9);
}

TEST(Projection, ExtractYawNegativePiOver2) {
  const beluga_estimation::Quaterniond q(Eigen::AngleAxisd(-M_PI / 2.0, beluga_estimation::Vector3d::UnitZ()));
  EXPECT_NEAR(beluga_estimation::ExtractYaw(q), -M_PI / 2.0, 1e-9);
}

TEST(Projection, ExtractYawPi) {
  const beluga_estimation::Quaterniond q(Eigen::AngleAxisd(M_PI, beluga_estimation::Vector3d::UnitZ()));
  EXPECT_NEAR(std::abs(beluga_estimation::ExtractYaw(q)), M_PI, 1e-9);
}

// ============================================================================
// InterpolateSE2Pose Tests
// ============================================================================

TEST(Interpolation, InterpolateSE2PoseAtZeroIsA) {
  const beluga_estimation::SE2d pose_a(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{1.0, 2.0});
  const beluga_estimation::SE2d pose_b(beluga_estimation::SO2d{M_PI / 2.0}, beluga_estimation::Vector2d{3.0, 4.0});

  const beluga_estimation::SE2d result = beluga_estimation::InterpolateSE2Pose(pose_a, pose_b, 0.0);
  EXPECT_NEAR(result.translation().x(), 1.0, 1e-9);
  EXPECT_NEAR(result.translation().y(), 2.0, 1e-9);
  EXPECT_NEAR(result.so2().log(), 0.0, 1e-9);
}

TEST(Interpolation, InterpolateSE2PoseAtOneIsB) {
  const beluga_estimation::SE2d pose_a(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{1.0, 2.0});
  const beluga_estimation::SE2d pose_b(beluga_estimation::SO2d{M_PI / 2.0}, beluga_estimation::Vector2d{3.0, 4.0});

  const beluga_estimation::SE2d result = beluga_estimation::InterpolateSE2Pose(pose_a, pose_b, 1.0);
  EXPECT_NEAR(result.translation().x(), 3.0, 1e-6);
  EXPECT_NEAR(result.translation().y(), 4.0, 1e-6);
  EXPECT_NEAR(result.so2().log(), M_PI / 2.0, 1e-6);
}

TEST(Interpolation, InterpolateSE2PoseAtHalfway) {
  const beluga_estimation::SE2d pose_a(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{0.0, 0.0});
  const beluga_estimation::SE2d pose_b(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{2.0, 0.0});

  const beluga_estimation::SE2d result = beluga_estimation::InterpolateSE2Pose(pose_a, pose_b, 0.5);
  EXPECT_NEAR(result.translation().x(), 1.0, 1e-9);
  EXPECT_NEAR(result.translation().y(), 0.0, 1e-9);
}

// ============================================================================
// InterpolateSPD Tests
// ============================================================================

TEST(Interpolation, InterpolateSPDAtZeroIsA) {
  const beluga_estimation::Matrix3d cov_a = beluga_estimation::Matrix3d::Identity() * 1.0;
  const beluga_estimation::Matrix3d cov_b = beluga_estimation::Matrix3d::Identity() * 4.0;

  const beluga_estimation::Matrix3d result = beluga_estimation::InterpolateSPD(cov_a, cov_b, 0.0);
  EXPECT_NEAR(result(0, 0), 1.0, 1e-9);
  EXPECT_NEAR(result(1, 1), 1.0, 1e-9);
  EXPECT_NEAR(result(2, 2), 1.0, 1e-9);
}

TEST(Interpolation, InterpolateSPDAtOneIsB) {
  const beluga_estimation::Matrix3d cov_a = beluga_estimation::Matrix3d::Identity() * 1.0;
  const beluga_estimation::Matrix3d cov_b = beluga_estimation::Matrix3d::Identity() * 4.0;

  const beluga_estimation::Matrix3d result = beluga_estimation::InterpolateSPD(cov_a, cov_b, 1.0);
  EXPECT_NEAR(result(0, 0), 4.0, 1e-6);
  EXPECT_NEAR(result(1, 1), 4.0, 1e-6);
  EXPECT_NEAR(result(2, 2), 4.0, 1e-6);
}

TEST(Interpolation, InterpolateSPDAtHalfwayIsGeometricMean) {
  const beluga_estimation::Matrix3d cov_a = beluga_estimation::Matrix3d::Identity() * 1.0;
  const beluga_estimation::Matrix3d cov_b = beluga_estimation::Matrix3d::Identity() * 4.0;

  // Log-Euclidean interpolation at alpha=0.5: geometric mean sqrt(1.0 * 4.0) = 2.0.
  // Linear interpolation would give (1.0 + 4.0) / 2 = 2.5, inflating uncertainty.
  const beluga_estimation::Matrix3d result = beluga_estimation::InterpolateSPD(cov_a, cov_b, 0.5);
  EXPECT_NEAR(result(0, 0), 2.0, 1e-9);
  EXPECT_NEAR(result(1, 1), 2.0, 1e-9);
  EXPECT_NEAR(result(2, 2), 2.0, 1e-9);
}

TEST(Project, ProjectsMaskedCoordinatesInOrder) {
  const beluga_estimation::Vector4d values{{1.0, 2.0, 3.0, 4.0}};

  const auto contiguous = beluga_estimation::Project(values, {false, true, true, false});
  ASSERT_EQ(contiguous.size(), 2);
  EXPECT_NEAR(contiguous(0), 2.0, 1e-12);
  EXPECT_NEAR(contiguous(1), 3.0, 1e-12);

  // Non-contiguous selections preserve their relative order.
  const auto non_contiguous = beluga_estimation::Project(values, {true, false, false, true});
  ASSERT_EQ(non_contiguous.size(), 2);
  EXPECT_NEAR(non_contiguous(0), 1.0, 1e-12);
  EXPECT_NEAR(non_contiguous(1), 4.0, 1e-12);

  const auto whole = beluga_estimation::Project(values, {true, true, true, true});
  EXPECT_EQ(whole.size(), 4);
  EXPECT_TRUE(whole.isApprox(values, 1e-12));
}

TEST(Project, VectorRejectsMaskLengthMismatch) {
  const beluga_estimation::Vector3d values{{1.0, 2.0, 3.0}};
  EXPECT_THROW(beluga_estimation::Project(values, {true, false}), std::runtime_error);
}

TEST(Project, MatrixProjectsMaskedRowsAndColumns) {
  beluga_estimation::Matrix3d values;
  values << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  const auto projected = beluga_estimation::Project(values, {true, false, true});
  ASSERT_EQ(projected.rows(), 2);
  ASSERT_EQ(projected.cols(), 2);
  EXPECT_NEAR(projected(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(projected(0, 1), 3.0, 1e-12);
  EXPECT_NEAR(projected(1, 0), 7.0, 1e-12);
  EXPECT_NEAR(projected(1, 1), 9.0, 1e-12);
}

TEST(Project, MatrixRejectsMaskLengthMismatch) {
  const beluga_estimation::Matrix3d values = beluga_estimation::Matrix3d::Identity();
  EXPECT_THROW(beluga_estimation::Project(values, {true, false}), std::runtime_error);
}

TEST(MakeProjectionMatrix, MapsMaskedCoordinatesInOrder) {
  const auto matrix = beluga_estimation::MakeProjectionMatrix<double>({true, false, false, true});

  ASSERT_EQ(matrix.rows(), 2);
  ASSERT_EQ(matrix.cols(), 4);
  EXPECT_NEAR(matrix(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(matrix(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(matrix(0, 2), 0.0, 1e-12);
  EXPECT_NEAR(matrix(0, 3), 0.0, 1e-12);
  EXPECT_NEAR(matrix(1, 0), 0.0, 1e-12);
  EXPECT_NEAR(matrix(1, 1), 0.0, 1e-12);
  EXPECT_NEAR(matrix(1, 2), 0.0, 1e-12);
  EXPECT_NEAR(matrix(1, 3), 1.0, 1e-12);

  // Multiplying by the matrix projects the same coordinates Project projects.
  const beluga_estimation::Vector4d values{{1.0, 2.0, 3.0, 4.0}};
  const beluga_estimation::VectorXd projected = matrix * values;
  const auto expected = beluga_estimation::Project(values, {true, false, false, true});
  EXPECT_TRUE(projected.isApprox(expected, 1e-12));
}

TEST(MakeProjectionMatrix, WholeMaskYieldsIdentity) {
  const auto matrix = beluga_estimation::MakeProjectionMatrix<float>({true, true, true});
  EXPECT_TRUE(matrix.isApprox(beluga_estimation::MatrixXf::Identity(3, 3), 1e-6f));
}

TEST(MakeProjectionMatrix, RejectsEmptySelection) {
  EXPECT_THROW(beluga_estimation::MakeProjectionMatrix<double>({false, false}), std::runtime_error);
}

}  // namespace

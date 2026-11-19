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

#include <chrono>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <range/v3/view/single.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/constraints/se2_constant_velocity_kinematic_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>

namespace {

using namespace beluga_estimation;

template <typename T>
class SE2ConstantVelocityKinematicConstraintTest : public ::testing::Test {};

using TestTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(SE2ConstantVelocityKinematicConstraintTest, TestTypes);

TYPED_TEST(SE2ConstantVelocityKinematicConstraintTest, Variables) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  SE2ConstantVelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Identifier{"vel_j", t1}, Matrix3<TypeParam>::Identity(), Matrix3<TypeParam>::Identity());

  EXPECT_TRUE(constraint.HasDependentVariables());
  EXPECT_TRUE(constraint.HasConditioningVariables());
  EXPECT_EQ(constraint.DependentVariables().size(), 2);
  EXPECT_EQ(constraint.DependentVariables()[0].name, "pose_j");
  EXPECT_EQ(constraint.DependentVariables()[1].name, "vel_j");
  EXPECT_EQ(constraint.ConditioningVariables().size(), 2);
  EXPECT_EQ(constraint.ConditioningVariables()[0].name, "pose_i");
  EXPECT_EQ(constraint.ConditioningVariables()[1].name, "vel_i");
}

TYPED_TEST(SE2ConstantVelocityKinematicConstraintTest, LogLikelihood_PeakAtExpected) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  const TypeParam dt = TypeParam{1.0};
  SE2ConstantVelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Identifier{"vel_j", t1}, Matrix3<TypeParam>::Identity(), Matrix3<TypeParam>::Identity());

  SE2<TypeParam> pose_i;
  Vector3<TypeParam> vel_i{{TypeParam{1}, TypeParam{0}, TypeParam{0}}};

  // With pose_j and vel_j at their expected values, likelihood should peak.
  SE2<TypeParam> pose_j_expected = pose_i * SE2<TypeParam>::exp(vel_i * dt);
  Vector3<TypeParam> vel_j_expected = vel_i;
  // Offset pose by 1 along x and velocity by 1 along x (Mahalanobis distance = 1 each, identity covariance).
  const Vector2<TypeParam> offset{TypeParam{1}, TypeParam{0}};
  SE2<TypeParam> pose_j_offset(SO2<TypeParam>{}, pose_j_expected.translation() + offset);
  Vector3<TypeParam> vel_j_offset = vel_i + Vector3<TypeParam>{{TypeParam{1}, TypeParam{0}, TypeParam{0}}};

  const TypeParam ll_expected = constraint.ComputeLogLikelihood(pose_j_expected, vel_j_expected, pose_i, vel_i);
  const TypeParam ll_offset = constraint.ComputeLogLikelihood(pose_j_offset, vel_j_offset, pose_i, vel_i);

  EXPECT_GT(ll_expected, ll_offset);
  // Difference equals -0.5 * (Mahalanobis_pose^2 + Mahalanobis_vel^2) = -0.5 * (1 + 1) = -1.0.
  EXPECT_NEAR(static_cast<double>(ll_expected - ll_offset), 1.0, 1e-4);
}

TYPED_TEST(SE2ConstantVelocityKinematicConstraintTest, TakeSamples_NearExpected) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  const TypeParam dt = TypeParam{1.0};
  SE2ConstantVelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Identifier{"vel_j", t1}, Matrix3<TypeParam>::Identity() * TypeParam{0.01},
      Matrix3<TypeParam>::Identity() * TypeParam{0.01});

  SE2<TypeParam> pose_i;
  Vector3<TypeParam> vel_i{{TypeParam{1}, TypeParam{0}, TypeParam{0}}};

  SE2<TypeParam> pose_j_sample;
  Vector3<TypeParam> vel_j_sample;

  RandomNumberGenerator rng(42);
  constraint.TakeSamples(
      pose_i, vel_i, ranges::views::single(beluga_estimation::Map<SE2<TypeParam>>(pose_j_sample.data())),
      ranges::views::single(beluga_estimation::Map<Vector3<TypeParam>>(vel_j_sample.data())), rng);

  const SE2<TypeParam> expected_pose = pose_i * SE2<TypeParam>::exp(vel_i * dt);
  EXPECT_NEAR(static_cast<double>((expected_pose.inverse() * pose_j_sample).log().norm()), 0.0, 0.5);
  EXPECT_NEAR(static_cast<double>((vel_j_sample - vel_i).norm()), 0.0, 0.5);
}

TYPED_TEST(SE2ConstantVelocityKinematicConstraintTest, CalcMean_CorrectPoseVelocityAndJacobian) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  const TypeParam dt = TypeParam{1.0};
  SE2ConstantVelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Identifier{"vel_j", t1}, Matrix3<TypeParam>::Identity(), Matrix3<TypeParam>::Identity());

  SE2<TypeParam> pose_i(SO2<TypeParam>{TypeParam{M_PI / 4.0}}, Vector2<TypeParam>{TypeParam{1}, TypeParam{2}});
  Vector3<TypeParam> vel_i{{TypeParam{1}, TypeParam{0}, TypeParam{0}}};

  std::vector<std_future::span<const TypeParam>> conditioning = {to_view(pose_i), to_view(vel_i)};

  // Mean output: 4 (SE2 ambient) + 3 (velocity) = 7 values
  Vector<TypeParam, 7> mean_buf;
  Matrix6<TypeParam> jac_out;

  constraint.CalcMean(conditioning, to_view(mean_buf), to_view(jac_out));

  const SE2<TypeParam> delta = SE2<TypeParam>::exp(vel_i * dt);
  const SE2<TypeParam> expected_pose = pose_i * delta;

  const beluga_estimation::Map<const SE2<TypeParam>> mean_pose(mean_buf.data());
  const beluga_estimation::Map<const Vector3<TypeParam>> mean_vel(mean_buf.data() + 4);

  EXPECT_NEAR(
      static_cast<double>(mean_pose.translation().x()), static_cast<double>(expected_pose.translation().x()), 1e-5);
  EXPECT_NEAR(
      static_cast<double>(mean_pose.translation().y()), static_cast<double>(expected_pose.translation().y()), 1e-5);
  EXPECT_NEAR(static_cast<double>(mean_pose.so2().log()), static_cast<double>(expected_pose.so2().log()), 1e-5);
  EXPECT_TRUE(mean_vel.isApprox(vel_i, TypeParam{1e-5}));

  // Cross-term d(pose_j)/d(vel_i) = dt * J_r(vel_i * dt), BCH: J_r(ξ) ≈ I - ad(ξ)/2.
  const Vector3<TypeParam> w = vel_i * dt;
  Matrix3<TypeParam> expected_cross = Matrix3<TypeParam>::Identity();
  expected_cross(0, 1) = w(2) / TypeParam{2};
  expected_cross(1, 0) = -w(2) / TypeParam{2};
  expected_cross(0, 2) = -w(1) / TypeParam{2};
  expected_cross(1, 2) = w(0) / TypeParam{2};
  expected_cross *= dt;

  EXPECT_TRUE((jac_out.template block<3, 3>(0, 0).isApprox(delta.inverse().Adj(), TypeParam{1e-5})));
  EXPECT_TRUE((jac_out.template block<3, 3>(3, 3).isApprox(Matrix3<TypeParam>::Identity(), TypeParam{1e-5})));
  EXPECT_TRUE((jac_out.template block<3, 3>(0, 3).isApprox(expected_cross, TypeParam{1e-5})));
  EXPECT_TRUE((jac_out.template block<3, 3>(3, 0).isZero(TypeParam{1e-5})));
}

TYPED_TEST(SE2ConstantVelocityKinematicConstraintTest, CalcCovariance_BlockDiagonal) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  Matrix3<TypeParam> Q_pose;
  Q_pose << TypeParam{1}, TypeParam{0.1}, TypeParam{0.2}, TypeParam{0.1}, TypeParam{2}, TypeParam{0.3}, TypeParam{0.2},
      TypeParam{0.3}, TypeParam{3};
  Matrix3<TypeParam> Q_vel = Matrix3<TypeParam>::Identity() * TypeParam{0.5};

  SE2ConstantVelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Identifier{"vel_j", t1}, Q_pose, Q_vel);

  Matrix6<TypeParam> cov_out;

  const std::vector<std_future::span<const TypeParam>> dummy;
  constraint.CalcCovariance(dummy, to_view(cov_out));

  // Process noise is block-diagonal: pose and velocity noise are independent.
  // The dt coupling belongs in F·P·F^T (via CalcMean's Jacobian), not here.
  EXPECT_TRUE((cov_out.template block<3, 3>(0, 0).isApprox(Q_pose, TypeParam{1e-6})));
  EXPECT_TRUE((cov_out.template block<3, 3>(3, 3).isApprox(Q_vel, TypeParam{1e-6})));
  EXPECT_TRUE((cov_out.template block<3, 3>(0, 3).isZero(TypeParam{1e-6})));
  EXPECT_TRUE((cov_out.template block<3, 3>(3, 0).isZero(TypeParam{1e-6})));
}

TYPED_TEST(SE2ConstantVelocityKinematicConstraintTest, CalcResidual_ZeroWhenAtExpected) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  const TypeParam dt = TypeParam{1.0};
  SE2ConstantVelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Identifier{"vel_j", t1}, Matrix3<TypeParam>::Identity(), Matrix3<TypeParam>::Identity());

  SE2<TypeParam> pose_i;
  Vector3<TypeParam> vel_i{{TypeParam{1}, TypeParam{0}, TypeParam{0}}};
  SE2<TypeParam> pose_j = pose_i * SE2<TypeParam>::exp(vel_i * dt);
  Vector3<TypeParam> vel_j = vel_i;

  std::vector<std_future::span<const TypeParam>> conditioning = {to_view(pose_i), to_view(vel_i)};
  std::vector<std_future::span<const TypeParam>> dependent = {to_view(pose_j), to_view(vel_j)};

  Vector6<TypeParam> residual;

  constraint.CalcResidual(dependent, conditioning, to_view(residual));

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  EXPECT_NEAR(static_cast<double>(residual.norm()), 0.0, 1e-5);
}

}  // namespace

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
#include <beluga_estimation/constraints/se2_velocity_kinematic_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>

namespace {

using namespace beluga_estimation;

template <typename T>
class SE2VelocityKinematicConstraintTest : public ::testing::Test {};

using TestTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(SE2VelocityKinematicConstraintTest, TestTypes);

TYPED_TEST(SE2VelocityKinematicConstraintTest, Variables) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  SE2VelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Matrix3<TypeParam>::Identity());

  EXPECT_TRUE(constraint.HasDependentVariables());
  EXPECT_TRUE(constraint.HasConditioningVariables());
  EXPECT_EQ(constraint.DependentVariables().size(), 1);
  EXPECT_EQ(constraint.DependentVariables()[0].name, "pose_j");
  EXPECT_EQ(constraint.ConditioningVariables().size(), 2);
  EXPECT_EQ(constraint.ConditioningVariables()[0].name, "pose_i");
  EXPECT_EQ(constraint.ConditioningVariables()[1].name, "vel_i");
}

TYPED_TEST(SE2VelocityKinematicConstraintTest, LogLikelihood_PeakAtExpected) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  SE2VelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Matrix3<TypeParam>::Identity());

  // pose_i at origin, velocity = 1 m/s forward
  SE2<TypeParam> pose_i;
  Vector3<TypeParam> vel{{TypeParam{1}, TypeParam{0}, TypeParam{0}}};

  // pose_j_expected = SE2::exp(vel * dt) = translation of 1 along x
  SE2<TypeParam> pose_j_expected(SO2<TypeParam>{}, Vector2<TypeParam>{TypeParam{1}, TypeParam{0}});
  // pose_j_offset = translation of 2 along x (Mahalanobis distance = 1)
  SE2<TypeParam> pose_j_offset(SO2<TypeParam>{}, Vector2<TypeParam>{TypeParam{2}, TypeParam{0}});

  const TypeParam ll_expected = constraint.ComputeLogLikelihood(pose_j_expected, pose_i, vel);
  const TypeParam ll_offset = constraint.ComputeLogLikelihood(pose_j_offset, pose_i, vel);

  EXPECT_GT(ll_expected, ll_offset);
  // Difference equals -0.5 * Mahalanobis^2 of the offset (= 1.0 with identity covariance)
  EXPECT_NEAR(static_cast<double>(ll_expected - ll_offset), 0.5, 1e-4);
}

TYPED_TEST(SE2VelocityKinematicConstraintTest, TakeSamples_NearExpected) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  SE2VelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Matrix3<TypeParam>::Identity() * TypeParam{0.01});

  SE2<TypeParam> pose_i;
  Vector3<TypeParam> vel{{TypeParam{1}, TypeParam{0}, TypeParam{0}}};
  SE2<TypeParam> pose_j_sample;

  RandomNumberGenerator rng(42);
  constraint.TakeSamples(
      pose_i, vel, ranges::views::single(beluga_estimation::Map<SE2<TypeParam>>(pose_j_sample.data())), rng);

  SE2<TypeParam> expected(SO2<TypeParam>{}, Vector2<TypeParam>{TypeParam{1}, TypeParam{0}});
  const auto error = (expected.inverse() * pose_j_sample).log();
  EXPECT_NEAR(static_cast<double>(error.norm()), 0.0, 0.5);
}

TYPED_TEST(SE2VelocityKinematicConstraintTest, CalcMean_CorrectPoseAndJacobian) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  const TypeParam dt = TypeParam{1.0};
  SE2VelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Matrix3<TypeParam>::Identity());

  SE2<TypeParam> pose_i(SO2<TypeParam>{TypeParam{M_PI / 4.0}}, Vector2<TypeParam>{TypeParam{1}, TypeParam{2}});
  Vector3<TypeParam> vel{{TypeParam{1}, TypeParam{0}, TypeParam{0}}};

  std::vector<std_future::span<const TypeParam>> conditioning = {to_view(pose_i), to_view(vel)};

  SE2<TypeParam> mean_out;
  Matrix<TypeParam, 3, 6> jac_out;

  constraint.CalcMean(conditioning, to_view(mean_out), to_view(jac_out));

  const SE2<TypeParam> delta = SE2<TypeParam>::exp(vel * dt);
  const SE2<TypeParam> expected_mean = pose_i * delta;

  EXPECT_NEAR(
      static_cast<double>(mean_out.translation().x()), static_cast<double>(expected_mean.translation().x()), 1e-5);
  EXPECT_NEAR(
      static_cast<double>(mean_out.translation().y()), static_cast<double>(expected_mean.translation().y()), 1e-5);
  EXPECT_NEAR(static_cast<double>(mean_out.so2().log()), static_cast<double>(expected_mean.so2().log()), 1e-5);

  // d(pose_j) / d(pose_i) = Ad(delta^{-1}). Ref: sola2018micro, Eq. (65) and (99).
  EXPECT_TRUE((jac_out.template block<3, 3>(0, 0).isApprox(delta.inverse().Adj(), TypeParam{1e-5})));
  // d(pose_j) / d(vel_i) = dt * Jr(vel_i * dt), BCH: Jr(ξ) ≈ I - ad(ξ)/2.
  const Vector3<TypeParam> w = vel * dt;
  Matrix3<TypeParam> expected_Jr = Matrix3<TypeParam>::Identity();
  expected_Jr(0, 1) = w(2) / TypeParam{2};
  expected_Jr(1, 0) = -w(2) / TypeParam{2};
  expected_Jr(0, 2) = -w(1) / TypeParam{2};
  expected_Jr(1, 2) = w(0) / TypeParam{2};
  EXPECT_TRUE((jac_out.template block<3, 3>(0, 3).isApprox(dt * expected_Jr, TypeParam{1e-5})));
}

TYPED_TEST(SE2VelocityKinematicConstraintTest, CalcResidual_ZeroWhenAtExpected) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  const TypeParam dt = TypeParam{1.0};
  SE2VelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Matrix3<TypeParam>::Identity());

  SE2<TypeParam> pose_i;
  Vector3<TypeParam> vel{{TypeParam{1}, TypeParam{0}, TypeParam{0}}};
  const SE2<TypeParam> delta = SE2<TypeParam>::exp(vel * dt);
  SE2<TypeParam> pose_j_expected = pose_i * delta;

  std::vector<std_future::span<const TypeParam>> conditioning = {to_view(pose_i), to_view(vel)};
  std::vector<SE2<TypeParam>> dependent = {pose_j_expected};

  Vector3<TypeParam> residual;

  constraint.CalcResidual(to_view(dependent), conditioning, to_view(residual));

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  EXPECT_NEAR(static_cast<double>(residual.norm()), 0.0, 1e-5);
}

TYPED_TEST(SE2VelocityKinematicConstraintTest, CalcResidual_JacobianMatchesBCH) {
  const TimePoint t0{};
  const TimePoint t1 = t0 + std::chrono::seconds(1);
  SE2VelocityKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i", t0}, Identifier{"vel_i", t0}, Identifier{"pose_j", t1},
      Matrix3<TypeParam>::Identity());

  SE2<TypeParam> pose_i(SO2<TypeParam>{TypeParam{0.1}}, Vector2<TypeParam>{TypeParam{1}, TypeParam{2}});
  Vector3<TypeParam> vel{{TypeParam{0.5}, TypeParam{0.3}, TypeParam{0.1}}};
  SE2<TypeParam> pose_j(SO2<TypeParam>{TypeParam{0.3}}, Vector2<TypeParam>{TypeParam{2}, TypeParam{1}});

  std::vector<std_future::span<const TypeParam>> conditioning = {to_view(pose_i), to_view(vel)};
  std::vector<SE2<TypeParam>> dependent = {pose_j};

  Vector3<TypeParam> residual;
  Matrix3<TypeParam> jac_out;

  constraint.CalcResidual(to_view(dependent), conditioning, to_view(residual), to_view(jac_out));

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  // First-order BCH approximation of Jr^{-1}(r) ≈ I + ad(r)/2.
  Matrix3<TypeParam> expected_jac = Matrix3<TypeParam>::Identity();
  const TypeParam half_theta = residual(2) / TypeParam{2};
  expected_jac(0, 1) = -half_theta;
  expected_jac(1, 0) = half_theta;
  expected_jac(0, 2) = residual(1) / TypeParam{2};
  expected_jac(1, 2) = -residual(0) / TypeParam{2};

  EXPECT_TRUE(jac_out.isApprox(expected_jac, TypeParam{1e-5}));
}

}  // namespace

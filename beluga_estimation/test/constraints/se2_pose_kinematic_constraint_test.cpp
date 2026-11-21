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

#include <vector>

#include <gtest/gtest.h>

#include <range/v3/view/single.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/constraints/se2_pose_kinematic_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>

namespace {

using namespace beluga_estimation;

template <typename T>
class SE2PoseKinematicConstraintTest : public ::testing::Test {};

using TestTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(SE2PoseKinematicConstraintTest, TestTypes);

TYPED_TEST(SE2PoseKinematicConstraintTest, Variables) {
  SE2PoseKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i"}, Identifier{"pose_j"}, Matrix3<TypeParam>::Identity());

  EXPECT_TRUE(constraint.HasDependentVariables());
  EXPECT_TRUE(constraint.HasConditioningVariables());
  EXPECT_EQ(constraint.DependentVariables().size(), 1);
  EXPECT_EQ(constraint.DependentVariables()[0].name, "pose_j");
  EXPECT_EQ(constraint.ConditioningVariables().size(), 1);
  EXPECT_EQ(constraint.ConditioningVariables()[0].name, "pose_i");
}

TYPED_TEST(SE2PoseKinematicConstraintTest, LogLikelihood_PeakAtExpected) {
  SE2PoseKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i"}, Identifier{"pose_j"}, Matrix3<TypeParam>::Identity());

  SE2<TypeParam> pose_i(SO2<TypeParam>{TypeParam{0.3}}, Vector2<TypeParam>{TypeParam{1}, TypeParam{2}});
  // With no velocity, pose_j_expected = pose_i
  SE2<TypeParam> pose_j_expected = pose_i;
  // Offset by 1 along x (Mahalanobis distance = 1 with identity covariance)
  SE2<TypeParam> pose_j_offset(SO2<TypeParam>{TypeParam{0.3}}, Vector2<TypeParam>{TypeParam{2}, TypeParam{2}});

  const TypeParam ll_expected = constraint.ComputeLogLikelihood(pose_j_expected, pose_i);
  const TypeParam ll_offset = constraint.ComputeLogLikelihood(pose_j_offset, pose_i);

  EXPECT_GT(ll_expected, ll_offset);
}

TYPED_TEST(SE2PoseKinematicConstraintTest, TakeSamples_NearExpected) {
  SE2PoseKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i"}, Identifier{"pose_j"}, Matrix3<TypeParam>::Identity() * TypeParam{0.01});

  SE2<TypeParam> pose_i(SO2<TypeParam>{TypeParam{0.5}}, Vector2<TypeParam>{TypeParam{3}, TypeParam{4}});
  SE2<TypeParam> pose_j_sample;

  RandomNumberGenerator rng(42);
  constraint.TakeSamples(
      pose_i, ranges::views::single(beluga_estimation::Map<SE2<TypeParam>>(pose_j_sample.data())), rng);

  const auto error = (pose_i.inverse() * pose_j_sample).log();
  EXPECT_NEAR(static_cast<double>(error.norm()), 0.0, 0.5);
}

TYPED_TEST(SE2PoseKinematicConstraintTest, CalcMean_CorrectPoseAndIdentityJacobian) {
  SE2PoseKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i"}, Identifier{"pose_j"}, Matrix3<TypeParam>::Identity());

  SE2<TypeParam> pose_i(SO2<TypeParam>{TypeParam{0.3}}, Vector2<TypeParam>{TypeParam{2}, TypeParam{1}});
  std::vector<SE2<TypeParam>> conditioning = {pose_i};

  SE2<TypeParam> mean_out;
  Matrix3<TypeParam> jac_out;

  constraint.CalcMean(to_view(conditioning), to_view(mean_out), to_view(jac_out));

  // With zero velocity, pose_j_expected = pose_i and Jacobian = Ad(identity) = I
  EXPECT_NEAR(static_cast<double>(mean_out.translation().x()), static_cast<double>(pose_i.translation().x()), 1e-5);
  EXPECT_NEAR(static_cast<double>(mean_out.translation().y()), static_cast<double>(pose_i.translation().y()), 1e-5);
  EXPECT_TRUE(jac_out.isApprox(Matrix3<TypeParam>::Identity(), TypeParam{1e-5}));
}

TYPED_TEST(SE2PoseKinematicConstraintTest, CalcResidual_JacobianMatchesBCH) {
  SE2PoseKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i"}, Identifier{"pose_j"}, Matrix3<TypeParam>::Identity());

  SE2<TypeParam> pose_i(SO2<TypeParam>{TypeParam{0.1}}, Vector2<TypeParam>{TypeParam{1}, TypeParam{2}});
  SE2<TypeParam> pose_j(SO2<TypeParam>{TypeParam{0.3}}, Vector2<TypeParam>{TypeParam{2}, TypeParam{1}});

  std::vector<SE2<TypeParam>> conditioning = {pose_i};
  std::vector<SE2<TypeParam>> dependent = {pose_j};

  Vector3<TypeParam> residual;
  Matrix3<TypeParam> jac_out;

  constraint.CalcResidual(to_view(dependent), to_view(conditioning), to_view(residual), to_view(jac_out));

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

TYPED_TEST(SE2PoseKinematicConstraintTest, CalcResidual_ZeroWhenAtExpected) {
  SE2PoseKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i"}, Identifier{"pose_j"}, Matrix3<TypeParam>::Identity());

  SE2<TypeParam> pose_i(SO2<TypeParam>{TypeParam{0.5}}, Vector2<TypeParam>{TypeParam{1}, TypeParam{2}});
  SE2<TypeParam> pose_j = pose_i;

  std::vector<SE2<TypeParam>> conditioning = {pose_i};
  std::vector<SE2<TypeParam>> dependent = {pose_j};

  Vector3<TypeParam> residual;
  constraint.CalcResidual(to_view(dependent), to_view(conditioning), to_view(residual));

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  EXPECT_NEAR(static_cast<double>(residual.norm()), 0.0, 1e-5);
}

TYPED_TEST(SE2PoseKinematicConstraintTest, WithDelta_ResidualZeroWhenAtExpected) {
  const SE2<TypeParam> delta(SO2<TypeParam>{TypeParam{M_PI / 4}}, Vector2<TypeParam>{TypeParam{1}, TypeParam{0}});

  SE2PoseKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i"}, Identifier{"pose_j"}, Matrix3<TypeParam>::Identity(), delta);

  SE2<TypeParam> pose_i(SO2<TypeParam>{TypeParam{0.3}}, Vector2<TypeParam>{TypeParam{2}, TypeParam{1}});
  SE2<TypeParam> pose_j = pose_i * delta;

  std::vector<SE2<TypeParam>> conditioning = {pose_i};
  std::vector<SE2<TypeParam>> dependent = {pose_j};

  Vector3<TypeParam> residual;
  constraint.CalcResidual(to_view(dependent), to_view(conditioning), to_view(residual));

  EXPECT_NEAR(static_cast<double>(residual.norm()), 0.0, 1e-5);
}

TYPED_TEST(SE2PoseKinematicConstraintTest, WithDelta_CalcMeanEqualsExpected) {
  const SE2<TypeParam> delta(SO2<TypeParam>{TypeParam{M_PI / 2}}, Vector2<TypeParam>{TypeParam{3}, TypeParam{1}});

  SE2PoseKinematicConstraint_<TypeParam> constraint(
      Identifier{"c"}, Identifier{"pose_i"}, Identifier{"pose_j"}, Matrix3<TypeParam>::Identity(), delta);

  SE2<TypeParam> pose_i(SO2<TypeParam>{TypeParam{0.1}}, Vector2<TypeParam>{TypeParam{1}, TypeParam{2}});
  std::vector<SE2<TypeParam>> conditioning = {pose_i};

  SE2<TypeParam> mean_out;
  constraint.CalcMean(to_view(conditioning), to_view(mean_out));

  const SE2<TypeParam> expected = pose_i * delta;
  EXPECT_NEAR(static_cast<double>(mean_out.translation().x()), static_cast<double>(expected.translation().x()), 1e-5);
  EXPECT_NEAR(static_cast<double>(mean_out.translation().y()), static_cast<double>(expected.translation().y()), 1e-5);
  EXPECT_NEAR(static_cast<double>(mean_out.so2().log()), static_cast<double>(expected.so2().log()), 1e-5);
}

}  // namespace

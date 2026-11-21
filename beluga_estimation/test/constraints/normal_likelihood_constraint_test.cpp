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
#include <optional>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/constraints/normal_likelihood_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>

namespace {

using namespace beluga_estimation;

// ============================================================================
// Typed Interface Tests (Eigen::Ref-based)
// ============================================================================

template <typename T>
class NormalLikelihoodConstraintTypedTest : public ::testing::Test {};

using TestTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(NormalLikelihoodConstraintTypedTest, TestTypes);

TYPED_TEST(NormalLikelihoodConstraintTypedTest, ConstructorAndIdentifiers) {
  Identifier id{"test_likelihood"};
  Identifier conditioning_id{"y"};

  Eigen::Vector<TypeParam, 2> z_obs{{10.0, 5.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(z_obs, cov);

  NormalLikelihoodConstraint_<TypeParam, 2> constraint(id, conditioning_id, dist);

  EXPECT_FALSE(constraint.HasDependentVariables());
  EXPECT_TRUE(constraint.HasConditioningVariables());
  EXPECT_EQ(constraint.DependentVariables().size(), 0);
  EXPECT_EQ(constraint.ConditioningVariables().size(), 1);
  EXPECT_EQ(constraint.ConditioningVariables()[0].name, "y");
}

TYPED_TEST(NormalLikelihoodConstraintTypedTest, ComputeLogLikelihood) {
  Identifier id{"test"};
  Identifier conditioning_id{"y"};

  Eigen::Vector<TypeParam, 2> z_obs{{10.0, 5.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(z_obs, cov);

  NormalLikelihoodConstraint_<TypeParam, 2> constraint(id, conditioning_id, dist);

  Eigen::Vector<TypeParam, 2> y_perfect{{10.0, 5.0}};
  Eigen::Vector<TypeParam, 2> y_offset{{11.0, 5.0}};

  auto log_likelihood_perfect = constraint.ComputeLogLikelihood(y_perfect);
  auto log_likelihood_offset = constraint.ComputeLogLikelihood(y_offset);

  constexpr TypeParam expected_normalization = static_cast<TypeParam>(-0.5 * 2 * 1.837877066409345);
  EXPECT_NEAR(log_likelihood_perfect, expected_normalization, 1e-5);
  EXPECT_NEAR(log_likelihood_offset, expected_normalization - TypeParam(0.5), 1e-5);
}

TYPED_TEST(NormalLikelihoodConstraintTypedTest, CalcCovariance) {
  Identifier id{"test"};
  Identifier conditioning_id{"y"};

  Eigen::Vector<TypeParam, 2> z_obs{{1.0, 2.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov;
  cov << 2.0, 0.5, 0.5, 1.0;
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(z_obs, cov);

  NormalLikelihoodConstraint_<TypeParam, 2> constraint(id, conditioning_id, dist);

  Eigen::Matrix<TypeParam, 2, 2, Eigen::RowMajor> covariance;
  constraint.CalcCovariance(covariance);

  EXPECT_NEAR(covariance(0, 0), 2.0, 1e-6);
  EXPECT_NEAR(covariance(0, 1), 0.5, 1e-6);
  EXPECT_NEAR(covariance(1, 0), 0.5, 1e-6);
  EXPECT_NEAR(covariance(1, 1), 1.0, 1e-6);
}

TYPED_TEST(NormalLikelihoodConstraintTypedTest, CalcResidual) {
  Identifier id{"test"};
  Identifier conditioning_id{"y"};

  Eigen::Vector<TypeParam, 2> z_obs{{10.0, 5.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(z_obs, cov);

  NormalLikelihoodConstraint_<TypeParam, 2> constraint(id, conditioning_id, dist);

  Eigen::Vector<TypeParam, 2> y{{12.0, 7.0}};
  Eigen::Vector<TypeParam, 2> residual;

  constraint.CalcResidual(y, residual);

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  // Residual = z_obs - y = [10, 5] - [12, 7] = [-2, -2]
  EXPECT_NEAR(residual[0], -2.0, 1e-6);
  EXPECT_NEAR(residual[1], -2.0, 1e-6);
}

TYPED_TEST(NormalLikelihoodConstraintTypedTest, CalcResidualWithJacobian) {
  Identifier id{"test"};
  Identifier conditioning_id{"y"};

  Eigen::Vector<TypeParam, 2> z_obs{{5.0, 3.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(z_obs, cov);

  NormalLikelihoodConstraint_<TypeParam, 2> constraint(id, conditioning_id, dist);

  Eigen::Vector<TypeParam, 2> y{{6.0, 4.0}};
  Eigen::Vector<TypeParam, 2> residual;
  Eigen::Matrix<TypeParam, 2, 2, Eigen::RowMajor> jacobian;

  constraint.CalcResidual(y, residual, jacobian);

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  // Residual = z_obs - y = [5, 3] - [6, 4] = [-1, -1]
  EXPECT_NEAR(residual[0], -1.0, 1e-6);
  EXPECT_NEAR(residual[1], -1.0, 1e-6);

  // Jacobian of the measurement model h(y) = y is identity.
  EXPECT_NEAR(jacobian(0, 0), 1.0, 1e-6);
  EXPECT_NEAR(jacobian(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(jacobian(1, 0), 0.0, 1e-6);
  EXPECT_NEAR(jacobian(1, 1), 1.0, 1e-6);
}

// Test dynamic dimension
TEST(NormalLikelihoodConstraintTypedTest, DynamicDimension) {
  Identifier id{"test_dynamic"};
  Identifier conditioning_id{"y"};

  constexpr int N = 3;
  Eigen::VectorXd z_obs(N);
  z_obs << 1.0, 2.0, 3.0;
  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(N, N);
  auto dist = std::make_shared<NormalDistribution_<double>>(z_obs, cov);

  NormalLikelihoodConstraint_<double> constraint(id, conditioning_id, dist);

  EXPECT_FALSE(constraint.HasDependentVariables());
  EXPECT_TRUE(constraint.HasConditioningVariables());

  Eigen::VectorXd y = z_obs;
  auto log_likelihood = constraint.ComputeLogLikelihood(y);

  constexpr double expected_normalization = -0.5 * 3 * 1.837877066409345;
  EXPECT_NEAR(log_likelihood, expected_normalization, 1e-5);
}

// ============================================================================
// Generic Interface Tests (span + view-based)
// ============================================================================

template <typename T>
class NormalLikelihoodConstraintViewTest : public ::testing::Test {};

TYPED_TEST_SUITE(NormalLikelihoodConstraintViewTest, TestTypes);

TYPED_TEST(NormalLikelihoodConstraintViewTest, ComputeLogLikelihood) {
  Identifier id{"test"};
  Identifier conditioning_id{"y"};

  Eigen::Vector<TypeParam, 2> z_obs{{10.0, 5.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(z_obs, cov);

  NormalLikelihoodConstraint_<TypeParam, 2> constraint(id, conditioning_id, dist);

  std::vector<Eigen::Vector<TypeParam, 2>> empty_dependent;
  std::vector<Eigen::Vector<TypeParam, 2>> conditioning = {{10.0, 5.0}};

  auto log_likelihood = constraint.ComputeLogLikelihood(to_view(empty_dependent), to_view(conditioning));

  constexpr TypeParam expected = static_cast<TypeParam>(-0.5 * 2 * 1.837877066409345);
  EXPECT_NEAR(log_likelihood, expected, 1e-5);
}

TYPED_TEST(NormalLikelihoodConstraintViewTest, TakeSamplesThrows) {
  Identifier id{"test"};
  Identifier conditioning_id{"y"};

  Eigen::Vector<TypeParam, 2> z_obs{{10.0, 5.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(z_obs, cov);

  NormalLikelihoodConstraint_<TypeParam, 2> constraint(id, conditioning_id, dist);

  std::vector<Eigen::Vector<TypeParam, 2>> conditioning = {{10.0, 5.0}};
  std::vector<std::vector<Eigen::Vector<TypeParam, 2>>> sampled = {{Eigen::Vector<TypeParam, 2>::Zero()}};

  RandomNumberGenerator generator(42);
  EXPECT_THROW(constraint.TakeSamples(to_view(conditioning), to_view(sampled), generator), std::runtime_error);
}

// ============================================================================
// Projection Tests
// ============================================================================

TEST(NormalLikelihoodConstraintProjectionTest, RejectsMismatchedProjectionRows) {
  Identifier id{"test"};
  Identifier conditioning_id{"velocity"};
  auto dist = std::make_shared<const NormalDistribution_<double>>(Vector2d::Zero(), Matrix2d::Identity());

  // Three rows cannot describe a two-dimensional measurement.
  EXPECT_THROW(
      (NormalLikelihoodConstraint_<double>(id, conditioning_id, dist, Matrix<double, 3, 3>::Zero())),
      std::runtime_error);
}

TEST(NormalLikelihoodConstraintProjectionTest, RejectsVariableSizeMismatchAtEvaluation) {
  Identifier id{"test"};
  Identifier conditioning_id{"velocity"};
  auto dist = std::make_shared<const NormalDistribution_<double>>(Vector1d::Zero(), Matrix1d::Identity());
  beluga_estimation::MatrixX<double> projection(1, 3);
  projection << 0.0, 0.0, 1.0;
  NormalLikelihoodConstraint_<double> constraint(id, conditioning_id, dist, projection);

  // The projection has three columns but the variable here has two.
  std::vector<Vector1d> empty;
  std::vector<Vector2d> too_short = {{1.0, 2.0}};
  Vector1d residual;

  EXPECT_THROW(constraint.CalcResidual(to_view(empty), to_view(too_short), to_view(residual)), std::runtime_error);
  EXPECT_THROW(constraint.ComputeLogLikelihood(to_view(empty), to_view(too_short)), std::runtime_error);
}

TEST(NormalLikelihoodConstraintProjectionTest, ResidualAndJacobianUseProjectionMatrix) {
  Identifier id{"test"};
  Identifier conditioning_id{"velocity"};
  // Fused measurement (1, -2) constraining two of four variable coordinates.
  auto dist = std::make_shared<const NormalDistribution_<double>>(Vector2d{{1.0, -2.0}}, Matrix2d::Identity());
  beluga_estimation::MatrixX<double> projection(2, 4);
  projection << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  NormalLikelihoodConstraint_<double> constraint(id, conditioning_id, dist, projection);

  std::vector<Vector1d> empty;
  std::vector<Vector4d> conditioning = {{0.5, 100.0, 100.0, -1.5}};
  Vector2d residual;
  Matrix<double, 2, 4> jacobian;

  constraint.CalcResidual(to_view(empty), to_view(conditioning), to_view(residual), to_view(jacobian));

  EXPECT_NEAR(residual(0), 1.0 - 0.5, 1e-12);
  EXPECT_NEAR(residual(1), -2.0 - -1.5, 1e-12);
  EXPECT_TRUE(jacobian.isApprox(projection, 1e-12));
}

TEST(NormalLikelihoodConstraintProjectionTest, LogLikelihoodUsesProjectedDistribution) {
  Identifier id{"test"};
  Identifier conditioning_id{"velocity"};
  // Only the third variable coordinate is observed; the others are far off.
  auto dist = std::make_shared<const NormalDistribution_<double>>(Vector1d{{0.5}}, Matrix1d::Identity());
  beluga_estimation::MatrixX<double> projection(1, 3);
  projection << 0.0, 0.0, 1.0;
  NormalLikelihoodConstraint_<double> constraint(id, conditioning_id, dist, projection);

  std::vector<Vector1d> empty;
  std::vector<Vector3d> conditioning = {{100.0, -100.0, 0.5}};

  const double log_likelihood = constraint.ComputeLogLikelihood(to_view(empty), to_view(conditioning));

  constexpr double kNormalization = -0.5 * 1 * 1.837877066409345;
  EXPECT_NEAR(log_likelihood, kNormalization, 1e-5);
}

TEST(NormalLikelihoodConstraintProjectionTest, SupportsWeightedObservations) {
  Identifier id{"test"};
  Identifier conditioning_id{"velocity"};
  // The measurement is the average of two variable coordinates, not a selection.
  auto dist = std::make_shared<const NormalDistribution_<double>>(Vector1d::Zero(), Matrix1d::Identity());
  beluga_estimation::MatrixX<double> projection(1, 2);
  projection << 0.5, 0.5;
  NormalLikelihoodConstraint_<double> constraint(id, conditioning_id, dist, projection);

  const Vector2d state{{1.0, 3.0}};

  Vector1d residual;
  Matrix<double, 1, 2> jacobian;
  constraint.CalcResidual(state, residual, jacobian);
  EXPECT_NEAR(residual(0), -0.5 * (1.0 + 3.0), 1e-12);
  EXPECT_TRUE(jacobian.isApprox(projection, 1e-12));

  const double log_likelihood = constraint.ComputeLogLikelihood(state);
  constexpr double kNormalization = -0.5 * 1 * 1.837877066409345;
  EXPECT_NEAR(log_likelihood, kNormalization - 0.5 * 4.0, 1e-5);
}

TEST(NormalLikelihoodConstraintProjectionTest, TypedInterfaceMatchesSpanInterface) {
  Identifier id{"test"};
  Identifier conditioning_id{"velocity"};
  auto dist = std::make_shared<const NormalDistribution_<double>>(Vector2d{{1.0, -2.0}}, Matrix2d::Identity());
  beluga_estimation::MatrixX<double> projection(2, 4);
  projection << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  NormalLikelihoodConstraint_<double> constraint(id, conditioning_id, dist, projection);

  const beluga_estimation::Vector4d state{{0.5, 100.0, 100.0, -1.5}};

  beluga_estimation::Vector2d typed_residual;
  beluga_estimation::Matrix<double, 2, 4> typed_jacobian;
  constraint.CalcResidual(state, typed_residual, typed_jacobian);

  std::vector<Vector1d> empty;
  std::vector<Vector4d> conditioning = {state};
  beluga_estimation::Vector2d span_residual;
  beluga_estimation::Matrix<double, 2, 4> span_jacobian;
  constraint.CalcResidual(to_view(empty), to_view(conditioning), to_view(span_residual), to_view(span_jacobian));

  EXPECT_TRUE(typed_residual.isApprox(span_residual, 1e-12));
  EXPECT_TRUE(typed_jacobian.isApprox(span_jacobian, 1e-12));

  const double typed_log_likelihood = constraint.ComputeLogLikelihood(state);
  const double span_log_likelihood = constraint.ComputeLogLikelihood(to_view(empty), to_view(conditioning));
  EXPECT_NEAR(typed_log_likelihood, span_log_likelihood, 1e-12);
}

}  // namespace

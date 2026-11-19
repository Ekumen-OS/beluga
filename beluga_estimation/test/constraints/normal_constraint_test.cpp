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
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/constraints/normal_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>

namespace {

using namespace beluga_estimation;

// ============================================================================
// Typed Interface Tests (Eigen::Ref-based)
// ============================================================================

template <typename T>
class NormalConstraintTypedTest : public ::testing::Test {};

using TestTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(NormalConstraintTypedTest, TestTypes);

TYPED_TEST(NormalConstraintTypedTest, ConstructorAndIdentifiers) {
  Identifier id{"test_binary"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  Eigen::Vector<TypeParam, 2> mean{{1.0, 2.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalConstraint_<TypeParam, 2> constraint(id, source_id, target_id, dist);

  EXPECT_TRUE(constraint.HasDependentVariables());
  EXPECT_TRUE(constraint.HasConditioningVariables());
  EXPECT_EQ(constraint.DependentVariables().size(), 1);
  EXPECT_EQ(constraint.DependentVariables()[0].name, "x_j");
  EXPECT_EQ(constraint.ConditioningVariables().size(), 1);
  EXPECT_EQ(constraint.ConditioningVariables()[0].name, "x_i");
}

TYPED_TEST(NormalConstraintTypedTest, ComputeLogLikelihood) {
  Identifier id{"test"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  Eigen::Vector<TypeParam, 2> mean{{1.0, 2.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalConstraint_<TypeParam, 2> constraint(id, source_id, target_id, dist);

  // Test perfect match: xj = xi + mean
  Eigen::Vector<TypeParam, 2> xi{{0.0, 0.0}};
  Eigen::Vector<TypeParam, 2> xj_perfect{{1.0, 2.0}};
  Eigen::Vector<TypeParam, 2> xj_offset{{2.0, 2.0}};

  auto log_likelihood_perfect = constraint.ComputeLogLikelihood(xj_perfect, xi);
  auto log_likelihood_offset = constraint.ComputeLogLikelihood(xj_offset, xi);

  constexpr TypeParam expected_normalization = static_cast<TypeParam>(-0.5 * 2 * 1.837877066409345);
  EXPECT_NEAR(log_likelihood_perfect, expected_normalization, 1e-5);
  EXPECT_NEAR(log_likelihood_offset, expected_normalization - TypeParam(0.5), 1e-5);
}

TYPED_TEST(NormalConstraintTypedTest, TakeSamples) {
  Identifier id{"test"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  Eigen::Vector<TypeParam, 2> mean{{1.0, 0.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity() * 0.01;
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalConstraint_<TypeParam, 2> constraint(id, source_id, target_id, dist);

  Eigen::Vector<TypeParam, 2> xi{{5.0, 3.0}};
  std::vector<Eigen::Vector<TypeParam, 2>> xj_samples(1);

  RandomNumberGenerator generator(42);
  constraint.TakeSamples(xi, xj_samples, generator);

  // Expected: xj = xi + mean = (5, 3) + (1, 0) = (6, 3)
  Eigen::Vector<TypeParam, 2> expected{{6.0, 3.0}};
  TypeParam error = (xj_samples[0] - expected).norm();

  // With covariance 0.01 (sigma = 0.1), samples should be close
  EXPECT_NEAR(error, 0.0, 0.5);
}

TYPED_TEST(NormalConstraintTypedTest, CalcMean) {
  Identifier id{"test"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  Eigen::Vector<TypeParam, 2> mean{{2.0, -1.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalConstraint_<TypeParam, 2> constraint(id, source_id, target_id, dist);

  Eigen::Vector<TypeParam, 2> xi{{3.0, 4.0}};
  Eigen::Vector<TypeParam, 2> xj_mean;

  constraint.CalcMean(xi, xj_mean);

  // Expected: xj = xi + mean = (3, 4) + (2, -1) = (5, 3)
  EXPECT_NEAR(xj_mean[0], 5.0, 1e-6);
  EXPECT_NEAR(xj_mean[1], 3.0, 1e-6);
}

TYPED_TEST(NormalConstraintTypedTest, CalcMeanWithJacobian) {
  Identifier id{"test"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  Eigen::Vector<TypeParam, 2> mean{{1.0, 1.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalConstraint_<TypeParam, 2> constraint(id, source_id, target_id, dist);

  Eigen::Vector<TypeParam, 2> xi{{0.0, 0.0}};
  Eigen::Vector<TypeParam, 2> xj_mean;
  Eigen::Matrix<TypeParam, 2, 2, Eigen::RowMajor> jacobian;

  constraint.CalcMean(xi, xj_mean, jacobian);

  // Jacobian should be identity for additive model
  EXPECT_NEAR(jacobian(0, 0), 1.0, 1e-6);
  EXPECT_NEAR(jacobian(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(jacobian(1, 0), 0.0, 1e-6);
  EXPECT_NEAR(jacobian(1, 1), 1.0, 1e-6);
}

TYPED_TEST(NormalConstraintTypedTest, CalcCovariance) {
  Identifier id{"test"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  Eigen::Vector<TypeParam, 2> mean{{0.0, 0.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov;
  cov << 1.0, 0.5, 0.5, 2.0;
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalConstraint_<TypeParam, 2> constraint(id, source_id, target_id, dist);

  Eigen::Matrix<TypeParam, 2, 2, Eigen::RowMajor> covariance;
  constraint.CalcCovariance(covariance);

  EXPECT_NEAR(covariance(0, 0), 1.0, 1e-6);
  EXPECT_NEAR(covariance(0, 1), 0.5, 1e-6);
  EXPECT_NEAR(covariance(1, 0), 0.5, 1e-6);
  EXPECT_NEAR(covariance(1, 1), 2.0, 1e-6);
}

TYPED_TEST(NormalConstraintTypedTest, CalcResidual) {
  Identifier id{"test"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  Eigen::Vector<TypeParam, 2> mean{{1.0, 2.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalConstraint_<TypeParam, 2> constraint(id, source_id, target_id, dist);

  Eigen::Vector<TypeParam, 2> xi{{3.0, 4.0}};
  Eigen::Vector<TypeParam, 2> xj{{5.0, 7.0}};
  Eigen::Vector<TypeParam, 2> residual;

  constraint.CalcResidual(xj, xi, residual);

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  // Expected: xj = (3, 4) + (1, 2) = (4, 6)
  // Residual: actual - expected = (5, 7) - (4, 6) = (1, 1)
  EXPECT_NEAR(residual[0], 1.0, 1e-6);
  EXPECT_NEAR(residual[1], 1.0, 1e-6);
}

TYPED_TEST(NormalConstraintTypedTest, CalcResidualWithJacobian) {
  Identifier id{"test"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  Eigen::Vector<TypeParam, 2> mean{{0.0, 0.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalConstraint_<TypeParam, 2> constraint(id, source_id, target_id, dist);

  Eigen::Vector<TypeParam, 2> xi{{0.0, 0.0}};
  Eigen::Vector<TypeParam, 2> xj{{0.0, 0.0}};
  Eigen::Vector<TypeParam, 2> residual;
  Eigen::Matrix<TypeParam, 2, 2, Eigen::RowMajor> jacobian;

  constraint.CalcResidual(xj, xi, residual, jacobian);

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  // Jacobian of residual w.r.t. xj is identity
  EXPECT_NEAR(jacobian(0, 0), 1.0, 1e-6);
  EXPECT_NEAR(jacobian(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(jacobian(1, 0), 0.0, 1e-6);
  EXPECT_NEAR(jacobian(1, 1), 1.0, 1e-6);
}

// Test dynamic dimension
TEST(NormalConstraintTypedTest, DynamicDimension) {
  Identifier id{"test_dynamic"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  constexpr int N = 5;
  Eigen::VectorXd mean = Eigen::VectorXd::Ones(N);
  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(N, N);
  auto dist = std::make_shared<NormalDistribution_<double>>(mean, cov);

  NormalConstraint_<double> constraint(id, source_id, target_id, dist);

  Eigen::VectorXd xi = Eigen::VectorXd::Zero(N);
  std::vector<Eigen::VectorXd> xj_samples(1, Eigen::VectorXd(N));

  RandomNumberGenerator generator(42);
  constraint.TakeSamples(xi, xj_samples, generator);

  Eigen::VectorXd expected = Eigen::VectorXd::Ones(N);
  double error = (xj_samples[0] - expected).norm();

  EXPECT_LT(error, 3.0);  // Increased tolerance for 5D random sample
}

// ============================================================================
// Generic Interface Tests (span + view-based)
// ============================================================================

template <typename T>
class NormalConstraintViewTest : public ::testing::Test {};

TYPED_TEST_SUITE(NormalConstraintViewTest, TestTypes);

TYPED_TEST(NormalConstraintViewTest, ComputeLogLikelihood) {
  Identifier id{"test"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  Eigen::Vector<TypeParam, 2> mean{{1.0, 2.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalConstraint_<TypeParam, 2> constraint(id, source_id, target_id, dist);

  std::vector<Eigen::Vector<TypeParam, 2>> conditioning = {{0.0, 0.0}};
  std::vector<Eigen::Vector<TypeParam, 2>> dependent = {{1.0, 2.0}};

  auto log_likelihood = constraint.ComputeLogLikelihood(to_view(dependent), to_view(conditioning));

  constexpr TypeParam expected = static_cast<TypeParam>(-0.5 * 2 * 1.837877066409345);
  EXPECT_NEAR(log_likelihood, expected, 1e-5);
}

TYPED_TEST(NormalConstraintViewTest, TakeSamples) {
  Identifier id{"test"};
  Identifier source_id{"x_i"};
  Identifier target_id{"x_j"};

  Eigen::Vector<TypeParam, 2> mean{{1.0, 0.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity() * 0.01;
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalConstraint_<TypeParam, 2> constraint(id, source_id, target_id, dist);

  std::vector<Eigen::Vector<TypeParam, 2>> conditioning = {{5.0, 3.0}};
  std::vector<std::vector<Eigen::Vector<TypeParam, 2>>> sampled = {{Eigen::Vector<TypeParam, 2>::Zero()}};

  RandomNumberGenerator generator(42);
  constraint.TakeSamples(to_view(conditioning), to_view(sampled), generator);

  Eigen::Vector<TypeParam, 2> expected{{6.0, 3.0}};
  TypeParam error = (sampled[0][0] - expected).norm();

  EXPECT_NEAR(error, 0.0, 0.5);
}

}  // namespace

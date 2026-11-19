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
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/constraints/normal_prior_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>

namespace {

using namespace beluga_estimation;

// ============================================================================
// Typed Interface Tests (Eigen::Ref-based)
// ============================================================================

template <typename T>
class NormalPriorConstraintTypedTest : public ::testing::Test {};

using TestTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(NormalPriorConstraintTypedTest, TestTypes);

TYPED_TEST(NormalPriorConstraintTypedTest, ConstructorAndIdentifiers) {
  Identifier id{"test_prior"};
  Identifier target_id{"x"};

  Eigen::Vector<TypeParam, 3> mean{{1.0, 2.0, 3.0}};
  Eigen::Matrix<TypeParam, 3, 3> cov = Eigen::Matrix<TypeParam, 3, 3>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 3>>(mean, cov);

  NormalPriorConstraint_<TypeParam, 3> constraint(id, target_id, dist);

  EXPECT_TRUE(constraint.HasDependentVariables());
  EXPECT_FALSE(constraint.HasConditioningVariables());
  EXPECT_EQ(constraint.DependentVariables().size(), 1);
  EXPECT_EQ(constraint.DependentVariables()[0].name, "x");
  EXPECT_EQ(constraint.ConditioningVariables().size(), 0);
}

TYPED_TEST(NormalPriorConstraintTypedTest, ComputeLogLikelihood) {
  Identifier id{"test"};
  Identifier target_id{"x"};

  Eigen::Vector<TypeParam, 2> mean{{5.0, -2.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalPriorConstraint_<TypeParam, 2> constraint(id, target_id, dist);

  Eigen::Vector<TypeParam, 2> x_perfect{{5.0, -2.0}};
  Eigen::Vector<TypeParam, 2> x_offset{{6.0, -2.0}};

  auto log_likelihood_perfect = constraint.ComputeLogLikelihood(x_perfect);
  auto log_likelihood_offset = constraint.ComputeLogLikelihood(x_offset);

  constexpr TypeParam expected_normalization = static_cast<TypeParam>(-0.5 * 2 * 1.837877066409345);
  EXPECT_NEAR(log_likelihood_perfect, expected_normalization, 1e-5);
  EXPECT_NEAR(log_likelihood_offset, expected_normalization - TypeParam(0.5), 1e-5);
}

TYPED_TEST(NormalPriorConstraintTypedTest, TakeSamples) {
  Identifier id{"test"};
  Identifier target_id{"x"};

  Eigen::Vector<TypeParam, 2> mean{{10.0, -5.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity() * 0.01;
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);
  NormalPriorConstraint_<TypeParam, 2> constraint(id, target_id, dist);

  std::vector<Eigen::Vector<TypeParam, 2>> x_samples(3);

  RandomNumberGenerator generator(42);
  constraint.TakeSamples(beluga_estimation::to_view(x_samples), generator);

  // All samples should be close to mean
  for (const auto& sample : x_samples) {
    TypeParam error = (sample - mean).norm();
    EXPECT_NEAR(error, 0.0, 0.5);
  }
}

TYPED_TEST(NormalPriorConstraintTypedTest, CalcMean) {
  Identifier id{"test"};
  Identifier target_id{"x"};

  Eigen::Vector<TypeParam, 3> mean{{1.0, 2.0, 3.0}};
  Eigen::Matrix<TypeParam, 3, 3> cov = Eigen::Matrix<TypeParam, 3, 3>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 3>>(mean, cov);

  NormalPriorConstraint_<TypeParam, 3> constraint(id, target_id, dist);

  Eigen::Vector<TypeParam, 3> x_mean;
  constraint.CalcMean(x_mean);

  EXPECT_NEAR(x_mean[0], 1.0, 1e-6);
  EXPECT_NEAR(x_mean[1], 2.0, 1e-6);
  EXPECT_NEAR(x_mean[2], 3.0, 1e-6);
}

TYPED_TEST(NormalPriorConstraintTypedTest, CalcCovariance) {
  Identifier id{"test"};
  Identifier target_id{"x"};

  Eigen::Vector<TypeParam, 2> mean{{0.0, 0.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov;
  cov << 2.0, 1.0, 1.0, 3.0;
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalPriorConstraint_<TypeParam, 2> constraint(id, target_id, dist);

  Eigen::Matrix<TypeParam, 2, 2, Eigen::RowMajor> covariance;
  constraint.CalcCovariance(covariance);

  EXPECT_NEAR(covariance(0, 0), 2.0, 1e-6);
  EXPECT_NEAR(covariance(0, 1), 1.0, 1e-6);
  EXPECT_NEAR(covariance(1, 0), 1.0, 1e-6);
  EXPECT_NEAR(covariance(1, 1), 3.0, 1e-6);
}

TYPED_TEST(NormalPriorConstraintTypedTest, CalcResidual) {
  Identifier id{"test"};
  Identifier target_id{"x"};

  Eigen::Vector<TypeParam, 2> mean{{3.0, 4.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalPriorConstraint_<TypeParam, 2> constraint(id, target_id, dist);

  Eigen::Vector<TypeParam, 2> x{{5.0, 7.0}};
  Eigen::Vector<TypeParam, 2> residual;

  constraint.CalcResidual(x, residual);

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  // Residual: actual - expected = (5, 7) - (3, 4) = (2, 3)
  EXPECT_NEAR(residual[0], 2.0, 1e-6);
  EXPECT_NEAR(residual[1], 3.0, 1e-6);
}

TYPED_TEST(NormalPriorConstraintTypedTest, CalcResidualWithJacobian) {
  Identifier id{"test"};
  Identifier target_id{"x"};

  Eigen::Vector<TypeParam, 2> mean{{0.0, 0.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalPriorConstraint_<TypeParam, 2> constraint(id, target_id, dist);

  Eigen::Vector<TypeParam, 2> x{{1.0, 2.0}};
  Eigen::Vector<TypeParam, 2> residual;
  Eigen::Matrix<TypeParam, 2, 2, Eigen::RowMajor> jacobian;

  constraint.CalcResidual(x, residual, jacobian);

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  // Jacobian is identity
  EXPECT_NEAR(jacobian(0, 0), 1.0, 1e-6);
  EXPECT_NEAR(jacobian(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(jacobian(1, 0), 0.0, 1e-6);
  EXPECT_NEAR(jacobian(1, 1), 1.0, 1e-6);
}

// Test dynamic dimension
TEST(NormalPriorConstraintTypedTest, DynamicDimension) {
  Identifier id{"test_dynamic"};
  Identifier target_id{"x"};

  constexpr int N = 4;
  Eigen::VectorXd mean(N);
  mean << 1.0, 2.0, 3.0, 4.0;
  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(N, N) * 0.1;
  auto dist = std::make_shared<NormalDistribution_<double>>(mean, cov);

  NormalPriorConstraint_<double> constraint(id, target_id, dist);

  EXPECT_FALSE(constraint.HasConditioningVariables());
  EXPECT_TRUE(constraint.HasDependentVariables());

  std::vector<Eigen::VectorXd> x_samples(5, Eigen::VectorXd(N));
  RandomNumberGenerator generator(42);

  constraint.TakeSamples(beluga_estimation::to_view(x_samples), generator);

  // Check that samples are reasonable
  for (const auto& sample : x_samples) {
    double error = (sample - mean).norm();
    EXPECT_LT(error, 2.0);
  }
}

// ============================================================================
// Generic Interface Tests (span + view-based)
// ============================================================================

template <typename T>
class NormalPriorConstraintViewTest : public ::testing::Test {};

TYPED_TEST_SUITE(NormalPriorConstraintViewTest, TestTypes);

TYPED_TEST(NormalPriorConstraintViewTest, ComputeLogLikelihood) {
  Identifier id{"test"};
  Identifier target_id{"x"};

  Eigen::Vector<TypeParam, 2> mean{{5.0, -2.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity();
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);

  NormalPriorConstraint_<TypeParam, 2> constraint(id, target_id, dist);

  std::vector<Eigen::Vector<TypeParam, 2>> dependent = {{5.0, -2.0}};
  std::vector<Eigen::Vector<TypeParam, 2>> empty_conditioning;

  auto log_likelihood = constraint.ComputeLogLikelihood(to_view(dependent), to_view(empty_conditioning));

  constexpr TypeParam expected = static_cast<TypeParam>(-0.5 * 2 * 1.837877066409345);
  EXPECT_NEAR(log_likelihood, expected, 1e-5);
}

TYPED_TEST(NormalPriorConstraintViewTest, TakeSamples) {
  Identifier id{"test"};
  Identifier target_id{"x"};

  Eigen::Vector<TypeParam, 2> mean{{10.0, -5.0}};
  Eigen::Matrix<TypeParam, 2, 2> cov = Eigen::Matrix<TypeParam, 2, 2>::Identity() * 0.01;
  auto dist = std::make_shared<NormalDistribution_<TypeParam, 2>>(mean, cov);
  NormalPriorConstraint_<TypeParam, 2> constraint(id, target_id, dist);

  std::vector<Eigen::Vector<TypeParam, 2>> empty_conditioning;
  std::vector<std::vector<Eigen::Vector<TypeParam, 2>>> sampled = {
      {Eigen::Vector<TypeParam, 2>::Zero(), Eigen::Vector<TypeParam, 2>::Zero(), Eigen::Vector<TypeParam, 2>::Zero()}};

  RandomNumberGenerator generator(42);
  constraint.TakeSamples(to_view(empty_conditioning), to_view(sampled), generator);

  // Check all samples are close to mean
  for (const auto& sample : sampled[0]) {
    TypeParam error = (sample - mean).norm();
    EXPECT_NEAR(error, 0.0, 0.5);
  }
}

}  // namespace

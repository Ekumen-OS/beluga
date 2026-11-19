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

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "beluga_estimation/distribution_test_utils.hpp"
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/core/distribution.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>

namespace {

using beluga_estimation::testing::ComputeEmpiricalStats;
using beluga_estimation::testing::DrawSamples;
using beluga_estimation::testing::kNumSamples;
using beluga_estimation::testing::kStatTolerance;
using beluga_estimation::testing::kTolerance;

// Fixture for a 2D Euclidean normal distribution (R^2).
// Uses Vector2d/Matrix2d to stay clearly in Euclidean space,
// distinct from SE(2) or SE(3) manifold distributions.
class NormalDistributionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mean << 1.0, 2.0;
    covariance << 1.0, 0.3, 0.3, 1.0;
  }

  beluga_estimation::Vector2d mean;
  beluga_estimation::Matrix2d covariance;
};

using NormalDistributionDeathTest = NormalDistributionTest;

// ============================================================================
// SECTION 1: Interface Consistency Tests
// Tests that virtual and typed interfaces produce identical results
// ============================================================================

TEST_F(NormalDistributionTest, TypedVsVirtualComputeMean) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  // Call virtual interface
  beluga_estimation::Vector2d mean_virtual;
  dist.ComputeMean(beluga_estimation::to_view(mean_virtual));

  // Call typed interface (directly pass vector, Ref wrapping happens internally)
  beluga_estimation::Vector2d mean_typed;
  dist.ComputeMean(mean_typed);

  EXPECT_TRUE(mean_virtual.isApprox(mean_typed, kTolerance));
  EXPECT_TRUE(mean_typed.isApprox(mean, kTolerance));
}

TEST_F(NormalDistributionTest, TypedVsVirtualComputeCovariance) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  // Call virtual interface
  beluga_estimation::Matrix2d cov_virtual;
  dist.ComputeCovariance(beluga_estimation::to_view(cov_virtual));

  // Call typed interface (directly pass matrix)
  beluga_estimation::Matrix2d cov_typed;
  dist.ComputeCovariance(cov_typed);

  EXPECT_TRUE(cov_virtual.isApprox(cov_typed, kTolerance));
  EXPECT_TRUE(cov_typed.isApprox(covariance, kTolerance));
}

TEST_F(NormalDistributionTest, TypedVsVirtualLogProbability) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  beluga_estimation::Vector2d test_value = mean;
  test_value[0] += 0.5;

  // Call virtual interface
  double log_prob_virtual = dist.LogProbability(beluga_estimation::to_view(test_value));

  // Call typed interface (directly pass vector)
  double log_prob_typed = dist.LogProbability(test_value);

  EXPECT_NEAR(log_prob_virtual, log_prob_typed, kTolerance);
}

TEST_F(NormalDistributionTest, TypedVsVirtualSample) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  // Draw samples via virtual interface
  beluga_estimation::RandomNumberGenerator rng1(42);
  auto samples = DrawSamples(dist, 1000, rng1);
  auto [mean, covariance] = ComputeEmpiricalStats(samples);

  // Draw samples via typed interface
  beluga_estimation::RandomNumberGenerator rng2(42);
  std::vector<beluga_estimation::Vector2d, Eigen::aligned_allocator<beluga_estimation::Vector2d>> other_samples(1000);
  dist.Sample(other_samples, rng2);
  auto [other_mean, other_covariance] = ComputeEmpiricalStats(other_samples);

  // Should produce similar statistics (not identical due to different random sequences)
  EXPECT_TRUE(mean.isApprox(other_mean, kStatTolerance));
  EXPECT_TRUE(covariance.isApprox(other_covariance, kStatTolerance));
}

// ============================================================================
// SECTION 2: Correctness Tests
// Tests mathematical properties and exact values
// ============================================================================

TEST_F(NormalDistributionTest, ComputeMean) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  beluga_estimation::Vector2d result = beluga_estimation::Vector2d::Zero();
  dist.ComputeMean(result);

  EXPECT_TRUE(result.isApprox(mean, kTolerance));
}

TEST_F(NormalDistributionTest, ComputeCovariance) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  beluga_estimation::Matrix2d result = beluga_estimation::Matrix2d::Zero();
  dist.ComputeCovariance(result);

  EXPECT_TRUE(result.isApprox(covariance, kTolerance));
}

TEST_F(NormalDistributionTest, LogProbabilityAbsoluteValue) {
  // Using an identity matrix makes the math easy to verify by hand
  beluga_estimation::NormalDistribution_<double, 2> identity_dist(
      beluga_estimation::Vector2d::Zero(), beluga_estimation::Matrix2d::Identity());

  beluga_estimation::Vector2d state = beluga_estimation::Vector2d::Zero();
  double log_prob = identity_dist.LogProbability(state);

  // For a 2D standard normal distribution evaluated at the mean,
  // the log pdf is exactly -log(2 * PI)
  double expected_log_prob = -std::log(2 * M_PI);

  EXPECT_NEAR(log_prob, expected_log_prob, kTolerance);
}

TEST_F(NormalDistributionTest, LogProbabilityPerturbed) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  // Log-probability at the mean must be the maximum
  double max_log_prob = dist.LogProbability(mean);

  // A perturbed point must be less likely than the mean
  beluga_estimation::Vector2d perturbed = mean;
  perturbed[0] += 1.0;
  double log_prob = dist.LogProbability(perturbed);
  EXPECT_LT(log_prob, max_log_prob);
  EXPECT_GT(log_prob, -100.0);
}

// ============================================================================
// SECTION 3: Statistical Validation
// Tests that samples match expected distributions
// ============================================================================

// Parameterized statistical test: verifies that Sample() produces samples whose
// empirical mean and covariance match the distribution parameters.
class NormalDistributionSampleTest
    : public ::testing::TestWithParam<std::pair<beluga_estimation::Vector2d, beluga_estimation::Matrix2d>> {};

TEST_P(NormalDistributionSampleTest, SampleMatchesDistributionParameters) {
  const auto& [expected_mean, expected_covariance] = GetParam();
  beluga_estimation::NormalDistribution_<double, 2> dist(expected_mean, expected_covariance);

  beluga_estimation::RandomNumberGenerator rng(42);
  const auto [sample_mean, sample_covariance] = ComputeEmpiricalStats(DrawSamples(dist, kNumSamples, rng));

  EXPECT_TRUE(sample_mean.isApprox(expected_mean, kStatTolerance));
  EXPECT_TRUE(sample_covariance.isApprox(expected_covariance, kStatTolerance));
}

INSTANTIATE_TEST_SUITE_P(
    MeanCovariancePairs,
    NormalDistributionSampleTest,
    ::testing::Values(
        std::make_pair(beluga_estimation::Vector2d{1.0, 2.0}, beluga_estimation::Matrix2d::Identity()),
        std::make_pair(
            beluga_estimation::Vector2d{3.0, 4.0},
            (beluga_estimation::Matrix2d() << 1.5, -0.3, -0.3, 1.5).finished()),
        std::make_pair(
            beluga_estimation::Vector2d{5.0, 6.0},
            (beluga_estimation::Matrix2d() << 2.0, 0.7, 0.7, 2.0).finished())));

// ============================================================================
// SECTION 4: Edge Cases and Death Tests
// ============================================================================

TEST_F(NormalDistributionDeathTest, AssertOnSizeMismatch) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  std::vector<double> result(1, 0.0);  // Intentionally wrong size
  auto span_result = beluga_estimation::std_future::span<double>{result};

  EXPECT_DEBUG_DEATH(dist.ComputeMean(span_result), "mean span must have size N");
}

TEST_F(NormalDistributionDeathTest, AssertOnCovarianceSizeMismatch) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  // Intentionally wrong size (3x3 instead of 2x2)
  beluga_estimation::Matrix3d bad_result = beluga_estimation::Matrix3d::Zero();
  auto bad_mdspan = beluga_estimation::std_future::mdspan<double, 2>{bad_result.data(), 3, 3};

  EXPECT_DEBUG_DEATH(dist.ComputeCovariance(bad_mdspan), "covariance mdspan must have rows == N");
}

TEST_F(NormalDistributionDeathTest, AssertOnLogProbabilitySizeMismatch) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  std::vector<double> bad_state(1, 0.0);  // 1D instead of 2D
  EXPECT_DEBUG_DEATH(
      dist.LogProbability(beluga_estimation::std_future::span<double>{bad_state}), "value span must have size N");
}

TEST_F(NormalDistributionDeathTest, AssertOnNonPositiveDefiniteCovariance) {
  // A matrix of all zeros is symmetric, but NOT strictly positive definite.
  beluga_estimation::Matrix2d bad_covariance = beluga_estimation::Matrix2d::Zero();

  // Positive definiteness is asserted on construction (debug builds only).
  EXPECT_DEBUG_DEATH(
      (beluga_estimation::NormalDistribution_<double, 2>(mean, bad_covariance)),
      "covariance matrix must be symmetric and positive definite");
}

// ============================================================================
// SECTION 5: Dynamic Dimension Tests
// Tests for the dynamic (Eigen::Dynamic) instantiation path.
// ============================================================================

TEST(NormalDistributionDynamicDeathTest, ConstructorAssertsOnNonSquareCovariance) {
  Eigen::VectorXd mean(2);
  mean << 1.0, 2.0;
  Eigen::MatrixXd covariance(2, 3);  // non-square
  covariance.setZero();
  EXPECT_DEBUG_DEATH((beluga_estimation::NormalDistribution(mean, covariance)), "covariance matrix must be square");
}

TEST(NormalDistributionDynamicDeathTest, ConstructorAssertsOnMeanCovarianceSizeMismatch) {
  Eigen::VectorXd mean(3);
  mean.setZero();
  Eigen::MatrixXd covariance(2, 2);  // size mismatch with mean
  covariance.setIdentity();
  EXPECT_DEBUG_DEATH(
      (beluga_estimation::NormalDistribution(mean, covariance)), "mean size must match covariance dimensions");
}

TEST(NormalDistributionDynamicTest, LogProbabilityCalculatesCorrectly) {
  Eigen::VectorXd mean(2);
  mean << 0.0, 0.0;
  Eigen::MatrixXd covariance(2, 2);
  covariance.setIdentity();

  beluga_estimation::NormalDistribution dist(mean, covariance);

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
  double log_prob = dist.LogProbability(state);

  double expected_log_prob = -std::log(2 * M_PI);
  EXPECT_NEAR(log_prob, expected_log_prob, kTolerance);
}

TEST(NormalDistributionDynamicTest, SampleMatchesDistributionParameters) {
  Eigen::VectorXd mean(2);
  mean << 3.0, 4.0;
  Eigen::MatrixXd covariance(2, 2);
  covariance << 1.5, -0.3, -0.3, 1.5;
  beluga_estimation::NormalDistribution dist(mean, covariance);

  beluga_estimation::RandomNumberGenerator rng(42);
  const auto [sample_mean, sample_covariance] = ComputeEmpiricalStats(DrawSamples(dist, kNumSamples, rng));

  EXPECT_TRUE(sample_mean.isApprox(mean, kStatTolerance));
  EXPECT_TRUE(sample_covariance.isApprox(covariance, kStatTolerance));
}

// ============================================================================
// SECTION 6: Marginalize Tests
// ============================================================================

TEST_F(NormalDistributionTest, MarginalizeSingleDimension) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  auto marginalized = dist.Marginalize(beluga_estimation::contiguous_slice::Span(0, 1));
  ASSERT_NE(marginalized, nullptr);

  beluga_estimation::Vector<double, 1> marg_mean;
  marginalized->ComputeMean(beluga_estimation::to_view(marg_mean));
  EXPECT_NEAR(marg_mean[0], mean[0], kTolerance);

  beluga_estimation::Matrix<double, 1, 1> marg_cov;
  marginalized->ComputeCovariance(marg_cov);
  EXPECT_NEAR(marg_cov(0, 0), covariance(0, 0), kTolerance);
}

TEST_F(NormalDistributionTest, MarginalizeMultipleSlices) {
  // 3D normal distribution for testing multi-slice marginalization.
  beluga_estimation::Vector3d mean3{1.0, 2.0, 3.0};
  beluga_estimation::Matrix3d cov3;
  cov3 << 1.0, 0.2, 0.1, 0.2, 2.0, 0.3, 0.1, 0.3, 3.0;
  beluga_estimation::NormalDistribution_<double, 3> dist(mean3, cov3);

  // Retain dimensions 0 and 2 (skip dim 1)
  beluga_estimation::contiguous_slice dims[]{
      beluga_estimation::contiguous_slice::Span(0, 1), beluga_estimation::contiguous_slice::Span(2, 1)};
  auto marginalized = dist.Marginalize(dims);
  ASSERT_NE(marginalized, nullptr);

  beluga_estimation::Vector2d marg_mean;
  marginalized->ComputeMean(beluga_estimation::to_view(marg_mean));
  EXPECT_NEAR(marg_mean[0], mean3[0], kTolerance);
  EXPECT_NEAR(marg_mean[1], mean3[2], kTolerance);

  beluga_estimation::Matrix2d marg_cov;
  marginalized->ComputeCovariance(marg_cov);
  EXPECT_NEAR(marg_cov(0, 0), cov3(0, 0), kTolerance);
  EXPECT_NEAR(marg_cov(0, 1), cov3(0, 2), kTolerance);
  EXPECT_NEAR(marg_cov(1, 0), cov3(2, 0), kTolerance);
  EXPECT_NEAR(marg_cov(1, 1), cov3(2, 2), kTolerance);
}

TEST_F(NormalDistributionTest, MarginalizeContiguousBlock) {
  beluga_estimation::NormalDistribution_<double, 2> dist(mean, covariance);

  auto marginalized = dist.Marginalize(beluga_estimation::contiguous_slice::Span(0, 2));
  ASSERT_NE(marginalized, nullptr);

  beluga_estimation::Vector2d marg_mean;
  marginalized->ComputeMean(beluga_estimation::to_view(marg_mean));
  EXPECT_TRUE(marg_mean.isApprox(mean, kTolerance));

  beluga_estimation::Matrix2d marg_cov;
  marginalized->ComputeCovariance(marg_cov);
  EXPECT_TRUE(marg_cov.isApprox(covariance, kTolerance));
}

}  // namespace

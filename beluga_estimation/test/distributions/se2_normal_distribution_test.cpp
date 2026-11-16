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

#include <array>
#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "beluga_estimation/distribution_test_utils.hpp"
#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/distributions/se2_normal_distribution.hpp>

namespace {

using beluga_estimation::testing::kTolerance;

// Fixture for SE(2) normal distribution tests
class SE2NormalDistributionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mean = beluga_estimation::SE2d(beluga_estimation::SO2d(M_PI / 4.0), beluga_estimation::Vector2d(5.0, -2.0));
    covariance = beluga_estimation::Matrix3d::Identity() * 0.05;
  }

  beluga_estimation::SE2d mean;
  beluga_estimation::Matrix3d covariance;
};

// ============================================================================
// SECTION 1: Interface Consistency Tests
// Tests that virtual and typed interfaces produce identical results
// ============================================================================

TEST_F(SE2NormalDistributionTest, TypedVsVirtualLogProbability) {
  beluga_estimation::SE2NormalDistribution dist(mean, covariance);

  // Create a test pose
  beluga_estimation::SE2<double> test_pose = mean;

  // Call virtual interface
  std::vector<double> test_vec(4, 0.0);
  beluga_estimation::Map<beluga_estimation::SE2<double>>(test_vec.data()) = test_pose;
  double log_prob_virtual = dist.LogProbability(beluga_estimation::std_future::span<const double>(test_vec.data(), 4));

  // Call typed interface
  double log_prob_typed = dist.LogProbability(test_pose);

  EXPECT_NEAR(log_prob_virtual, log_prob_typed, kTolerance);
}

TEST_F(SE2NormalDistributionTest, TypedVsVirtualSample) {
  beluga_estimation::SE2NormalDistribution dist(mean, covariance);

  // Draw samples via typed interface
  beluga_estimation::RandomNumberGenerator rng1(42);
  auto samples_typed = dist.Sample(100, rng1);

  // Draw samples via virtual interface (through span)
  beluga_estimation::RandomNumberGenerator rng2(42);
  std::vector<std::vector<double>> sample_buffers(100, std::vector<double>(4, 0.0));
  std::vector<beluga_estimation::std_future::span<double>> spans;
  for (auto& buf : sample_buffers) {
    spans.emplace_back(buf.data(), 4);
  }
  beluga_estimation::std_future::span<beluga_estimation::std_future::span<double>> sample_view(
      spans.data(), spans.size());
  dist.Sample(beluga_estimation::to_view(spans), rng2);

  // Compare first few samples (should be identical with same seed)
  for (size_t i = 0; i < 10; ++i) {
    const auto& typed_data = samples_typed[i].data();
    for (size_t j = 0; j < 4; ++j) {
      EXPECT_NEAR(typed_data[j], sample_buffers[i][j], kTolerance);
    }
  }
}

// ============================================================================
// SECTION 2: Correctness Tests
// Tests mathematical properties and exact values
// ============================================================================

TEST_F(SE2NormalDistributionTest, ConstructionAndQuery) {
  // Definition of Done: Instantiation of an SE(2) Normal Distribution
  beluga_estimation::SE2d test_mean(beluga_estimation::SO2d(M_PI / 2.0), beluga_estimation::Vector2d(1.0, 2.0));
  beluga_estimation::Matrix3d test_covariance = beluga_estimation::Matrix3d::Identity() * 0.1;

  beluga_estimation::SE2NormalDistribution dist(test_mean, test_covariance);

  // Definition of Done: Successful query of its mean
  std::vector<double> mean_out_vec(4, 0.0);
  beluga_estimation::std_future::span<double> mean_out(mean_out_vec.data(), mean_out_vec.size());
  dist.ComputeMean(mean_out);

  // mean_out should be [cos(pi/2), sin(pi/2), 1.0, 2.0]
  EXPECT_NEAR(mean_out[0], 0.0, kTolerance);
  EXPECT_NEAR(mean_out[1], 1.0, kTolerance);
  EXPECT_NEAR(mean_out[2], 1.0, kTolerance);
  EXPECT_NEAR(mean_out[3], 2.0, kTolerance);

  // Definition of Done: Successful query of its covariance
  std::vector<double> cov_out_vec(9, 0.0);
  beluga_estimation::std_future::mdspan<double, 2> cov_out(cov_out_vec.data(), 3, 3);
  dist.ComputeCovariance(cov_out);

  EXPECT_NEAR(cov_out(0, 0), 0.1, kTolerance);
  EXPECT_NEAR(cov_out(0, 1), 0.0, kTolerance);
  EXPECT_NEAR(cov_out(1, 1), 0.1, kTolerance);
  EXPECT_NEAR(cov_out(2, 2), 0.1, kTolerance);
}

TEST_F(SE2NormalDistributionTest, Sample) {
  beluga_estimation::SE2NormalDistribution dist(mean, covariance);

  // Sample
  beluga_estimation::RandomNumberGenerator rng(42);  // deterministic seed
  auto samples = dist.Sample(2, rng);

  ASSERT_EQ(samples.size(), 2);

  // We ensure the samples are valid SE2 elements (rotation part normalized)
  for (const auto& sample : samples) {
    const double* data = sample.data();
    EXPECT_NEAR(data[0] * data[0] + data[1] * data[1], 1.0, kTolerance);
  }
}

TEST_F(SE2NormalDistributionTest, LogProbability) {
  beluga_estimation::SE2NormalDistribution dist(mean, covariance);

  // The log-prob of a sample should be negative and relatively close to the mean's log-prob
  std::vector<double> mean_vec(4, 0.0);
  beluga_estimation::std_future::span<double> mean_out(mean_vec.data(), mean_vec.size());
  dist.ComputeMean(mean_out);
  double max_log_prob = dist.LogProbability(mean_out);

  // Provide a perturbed sample
  std::vector<double> sample_vec = mean_vec;
  sample_vec[2] += 0.5;  // Offset in X
  sample_vec[3] -= 0.5;  // Offset in Y
  beluga_estimation::std_future::span<double> sample_out(sample_vec.data(), sample_vec.size());

  double log_prob = dist.LogProbability(sample_out);

  EXPECT_LT(log_prob, max_log_prob);  // A sample is less likely than the exact mean
  EXPECT_GT(log_prob, -100.0);        // But should be a reasonable value, not infinitely unlikely
}

TEST_F(SE2NormalDistributionTest, LogProbabilityAtMean) {
  beluga_estimation::SE2NormalDistribution dist(mean, covariance);

  // Log-probability at mean should be higher than at perturbed poses
  double log_prob_at_mean = dist.LogProbability(mean);

  // Create a perturbed pose
  beluga_estimation::SE2<double> perturbed =
      mean * beluga_estimation::SE2<double>::exp(beluga_estimation::Vector3d(0.1, 0.1, 0.1));
  double log_prob_perturbed = dist.LogProbability(perturbed);

  EXPECT_GT(log_prob_at_mean, log_prob_perturbed);
}

TEST_F(SE2NormalDistributionTest, MarginalizeReturnsNullptr) {
  beluga_estimation::SE2NormalDistribution dist(mean, covariance);

  // SE(2) tangent space is 3D. Full marginalization (all 3 dimensions) is
  // supported and returns the distribution. Partial marginalization of a
  // subset (e.g. 2 of 3 dimensions) is not supported and returns nullptr.
  auto result = dist.Marginalize(beluga_estimation::contiguous_slice::Span(0, 2));
  EXPECT_EQ(result, nullptr);

  beluga_estimation::contiguous_slice dims[]{beluga_estimation::contiguous_slice::Span(0, 2)};
  auto result_span = dist.Marginalize(dims);
  EXPECT_EQ(result_span, nullptr);
}

// ============================================================================
// SECTION 3: Residual Calculation
// ============================================================================

TEST_F(SE2NormalDistributionTest, CalculateResidual) {
  beluga_estimation::SE2NormalDistribution dist(mean, covariance);

  // Residual at the mean is zero.
  beluga_estimation::Vector3d residual_at_mean = dist.CalculateResidual(mean);
  EXPECT_NEAR(residual_at_mean.norm(), 0.0, kTolerance);

  // Residual at a displaced pose is non-zero.
  beluga_estimation::SE2<double> displaced =
      mean * beluga_estimation::SE2<double>::exp(beluga_estimation::Vector3d(0.5, 0.0, 0.0));
  beluga_estimation::Vector3d residual_at_displaced = dist.CalculateResidual(displaced);
  EXPECT_GT(residual_at_displaced.norm(), 0.0);

  // Matches manual computation: (mean^{-1} * displaced).log()
  beluga_estimation::Vector3d expected = (mean.inverse() * displaced).log();
  EXPECT_TRUE(residual_at_displaced.isApprox(expected, kTolerance));
}

}  // namespace

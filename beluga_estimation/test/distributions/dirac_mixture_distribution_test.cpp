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

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/take.hpp>

#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/distributions/dirac_mixture_distribution.hpp>
#include <beluga_estimation/mocks/mock_distribution.hpp>
#include "beluga_estimation/distribution_test_utils.hpp"

namespace {

using Vec1 = Eigen::Matrix<double, 1, 1>;
using Vec2 = Eigen::Matrix<double, 2, 1>;
using beluga_estimation::testing::kTolerance;

// ---------------------------------------------------------------------------
// Fixture: 1D points [0, 1, 2], unnormalized weights [1, 2, 1] → p = [0.25, 0.5, 0.25].
//
//   ComputeMean:       sum_i p_i * c_i = 0*0.25 + 1*0.5 + 2*0.25 = 1.0
//   ComputeCovariance: sum_i p_i*(c_i - mean)^2 = 0.25 + 0 + 0.25 = 0.5
// ---------------------------------------------------------------------------
class DiracMixtureDistributionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    cov_data_.assign(1, 0.0);
    cov_ = beluga_estimation::std_future::mdspan<double, 2>(cov_data_.data(), 1, 1);
    dist_.ComputeCovariance(cov_);
  }

  beluga_estimation::DiracMixtureNDistribution<1> dist_{{Vec1(0.0), Vec1(1.0), Vec1(2.0)}, {1.0, 2.0, 1.0}};
  beluga_estimation::RandomNumberGenerator rng_{42};
  std::vector<double> cov_data_;
  beluga_estimation::std_future::mdspan<double, 2> cov_;
};

// ---------------------------------------------------------------------------
// Fixture: 2D points [(0,1), (2,0)], equal weights → p = [0.5, 0.5].
//
//   ComputeMean:       (1.0, 0.5)
//   ComputeCovariance: [[1.0, -0.5], [-0.5, 0.25]]
// ---------------------------------------------------------------------------
class DiracMixtureDistribution2DTest : public ::testing::Test {
 protected:
  const std::vector<Vec2> pts_{Vec2(0.0, 1.0), Vec2(2.0, 0.0)};
  beluga_estimation::DiracMixtureNDistribution<2> dist_{pts_, {1.0, 1.0}};
  beluga_estimation::RandomNumberGenerator rng_{42};
};

// ============================================================================
// SECTION 1: Interface Consistency Tests
// Tests that virtual and typed interfaces produce identical results
// ============================================================================

TEST_F(DiracMixtureDistributionTest, TypedVsVirtualComputeMean) {
  // Call virtual interface
  std::vector<double> mean_virtual_data(1, 0.0);
  beluga_estimation::std_future::span<double> mean_virtual(mean_virtual_data.data(), 1);
  dist_.ComputeMean(mean_virtual);

  // Call typed interface (directly pass vector)
  Vec1 mean_typed;
  dist_.ComputeMean(mean_typed);

  EXPECT_NEAR(mean_virtual[0], mean_typed[0], kTolerance);
}

TEST_F(DiracMixtureDistributionTest, TypedVsVirtualComputeCovariance) {
  // Call virtual interface
  std::vector<double> cov_virtual_data(1, 0.0);
  beluga_estimation::std_future::mdspan<double, 2> cov_virtual(cov_virtual_data.data(), 1, 1);
  dist_.ComputeCovariance(cov_virtual);

  // Call typed interface (directly pass matrix)
  Eigen::Matrix<double, 1, 1> cov_typed;
  dist_.ComputeCovariance(cov_typed);

  EXPECT_NEAR(cov_virtual(0, 0), cov_typed(0, 0), kTolerance);
}

TEST_F(DiracMixtureDistributionTest, TypedVsVirtualLogProbability) {
  // Both should return -infinity
  std::vector<double> test_value{1.0};
  double log_prob_virtual =
      dist_.LogProbability(beluga_estimation::std_future::span<const double>(test_value.data(), 1));

  Vec1 test_vec(1.0);
  double log_prob_typed = dist_.LogProbability(test_vec);

  EXPECT_EQ(log_prob_virtual, -std::numeric_limits<double>::infinity());
  EXPECT_EQ(log_prob_typed, -std::numeric_limits<double>::infinity());
}

TEST_F(DiracMixtureDistributionTest, TypedVsVirtualSample) {
  // Draw samples via both interfaces with same seed - should produce same results
  const auto& pts = dist_.Points();

  // Virtual interface
  beluga_estimation::RandomNumberGenerator rng1(123);
  std::vector<double> s0_virtual(1, 0.0);
  std::vector<double> s1_virtual(1, 0.0);
  std::vector<beluga_estimation::std_future::span<double>> spans_virtual = {
      beluga_estimation::std_future::span<double>(s0_virtual.data(), 1),
      beluga_estimation::std_future::span<double>(s1_virtual.data(), 1)};
  beluga_estimation::std_future::span<beluga_estimation::std_future::span<double>> out_virtual(spans_virtual.data(), 2);
  dist_.Sample(out_virtual, rng1);

  // Typed interface
  beluga_estimation::RandomNumberGenerator rng2(123);
  std::vector<Vec1> samples_typed(2);
  dist_.Sample(beluga_estimation::to_view(samples_typed), rng2);

  // Should produce the same samples
  EXPECT_NEAR(s0_virtual[0], samples_typed[0][0], kTolerance);
  EXPECT_NEAR(s1_virtual[0], samples_typed[1][0], kTolerance);
}

// ============================================================================
// SECTION 2: Construction and Properties
// ============================================================================

TEST_F(DiracMixtureDistributionTest, ProbabilitiesSumToOne) {
  const auto& probs = dist_.Probabilities();
  EXPECT_NEAR(std::accumulate(probs.begin(), probs.end(), 0.0), 1.0, 1e-9);
}

TEST_F(DiracMixtureDistributionTest, ProbabilitiesAreNormalized) {
  const auto& probs = dist_.Probabilities();
  EXPECT_NEAR(probs[0], 0.25, 1e-9);
  EXPECT_NEAR(probs[1], 0.50, 1e-9);
  EXPECT_NEAR(probs[2], 0.25, 1e-9);
}

TEST_F(DiracMixtureDistributionTest, UnnormalizedWeightsGiveSameProbabilities) {
  beluga_estimation::DiracMixtureNDistribution<1> dist_scaled{{Vec1(0.0), Vec1(1.0), Vec1(2.0)}, {2.0, 4.0, 2.0}};
  const auto& p1 = dist_.Probabilities();
  const auto& p2 = dist_scaled.Probabilities();
  for (std::size_t i = 0; i < p1.size(); ++i) {
    EXPECT_NEAR(p1[i], p2[i], 1e-9);
  }
}

TEST_F(DiracMixtureDistributionTest, PointsAreStored) {
  const auto& pts = dist_.Points();
  ASSERT_EQ(pts.size(), 3u);
  EXPECT_NEAR(pts[0][0], 0.0, 1e-9);
  EXPECT_NEAR(pts[1][0], 1.0, 1e-9);
  EXPECT_NEAR(pts[2][0], 2.0, 1e-9);
}

// ============================================================================
// SECTION 3: ComputeMean Tests
// ============================================================================

TEST_F(DiracMixtureDistributionTest, ComputeMean) {
  std::vector<double> mean_data(1, 0.0);
  beluga_estimation::std_future::span<double> mean(mean_data.data(), 1);
  dist_.ComputeMean(mean);
  EXPECT_NEAR(mean[0], 1.0, 1e-9);
}

TEST(DiracMixtureDistributionMeanTest, EquiprobablePoints) {
  beluga_estimation::DiracMixtureNDistribution<1> d{{Vec1(0.0), Vec1(10.0)}, {1.0, 1.0}};
  std::vector<double> mean_data(1, 0.0);
  beluga_estimation::std_future::span<double> mean(mean_data.data(), 1);
  d.ComputeMean(mean);
  EXPECT_NEAR(mean[0], 5.0, 1e-9);
}

// ============================================================================
// SECTION 4: ComputeCovariance Tests
// ============================================================================

TEST_F(DiracMixtureDistributionTest, ComputeCovarianceValue) {
  EXPECT_NEAR(cov_(0, 0), 0.5, 1e-9);
}

// ============================================================================
// SECTION 5: LogProbability Tests
// ============================================================================

TEST_F(DiracMixtureDistributionTest, LogProbabilityReturnsNegativeInfinity) {
  std::vector<double> counts{1.0};
  beluga_estimation::std_future::span<const double> value(counts.data(), 1);
  EXPECT_EQ(dist_.LogProbability(value), -std::numeric_limits<double>::infinity());
}

// ============================================================================
// SECTION 6: Sample Tests
// ============================================================================

TEST_F(DiracMixtureDistributionTest, SampleReturnsValidPoints) {
  const auto& pts = dist_.Points();
  std::vector<double> s0(1, 0.0);
  std::vector<double> s1(1, 0.0);
  std::vector<beluga_estimation::std_future::span<double>> spans = {
      beluga_estimation::std_future::span<double>(s0.data(), 1),
      beluga_estimation::std_future::span<double>(s1.data(), 1)};
  beluga_estimation::std_future::span<beluga_estimation::std_future::span<double>> out(spans.data(), 2);
  dist_.Sample(out, rng_);
  const auto has_value = [&](double v) {
    return std::any_of(pts.begin(), pts.end(), [v](const Vec1& c) { return std::abs(c[0] - v) < 1e-9; });
  };
  EXPECT_TRUE(has_value(s0[0]));
  EXPECT_TRUE(has_value(s1[0]));
}

TEST_F(DiracMixtureDistributionTest, SampleWithDeterministicRng) {
  // Verify sampling works correctly with a deterministic RNG.
  beluga_estimation::RandomNumberGenerator rng(123);
  const auto& pts = dist_.Points();
  auto samples = dist_.Sample(1, rng);
  ASSERT_EQ(samples.size(), 1);
  EXPECT_TRUE(std::any_of(pts.begin(), pts.end(), [&](const Vec1& c) { return (samples[0] - c).norm() < 1e-9; }));
}

TEST_F(DiracMixtureDistributionTest, SamplePointsAreValid) {
  const auto& pts = dist_.Points();
  auto samples = dist_.Sample(rng_) | ranges::views::take(1000) | ranges::to<std::vector>();
  for (const auto& s : samples) {
    EXPECT_TRUE(std::any_of(pts.begin(), pts.end(), [&](const Vec1& c) { return (s - c).norm() < 1e-9; }));
  }
}

TEST_F(DiracMixtureDistributionTest, SampleStatisticalFrequencies) {
  constexpr std::size_t kDraws = 100000;
  std::size_t counts[3] = {0, 0, 0};
  for (const auto& s : dist_.Sample(rng_) | ranges::views::take(kDraws)) {
    const int idx = static_cast<int>(std::round(s[0]));
    ++counts[idx];
  }
  EXPECT_NEAR(static_cast<double>(counts[0]) / kDraws, 0.25, 0.01);
  EXPECT_NEAR(static_cast<double>(counts[1]) / kDraws, 0.50, 0.01);
  EXPECT_NEAR(static_cast<double>(counts[2]) / kDraws, 0.25, 0.01);
}

TEST(DiracMixtureDistributionSampleTest, SinglePointAlwaysSampled) {
  beluga_estimation::DiracMixtureNDistribution<1> d{{Vec1(42.0)}, {1.0}};
  beluga_estimation::RandomNumberGenerator rng(0);
  auto samples = d.Sample(rng) | ranges::views::take(50) | ranges::to<std::vector>();
  for (const auto& s : samples) {
    EXPECT_NEAR(s[0], 42.0, 1e-9);
  }
}

// ============================================================================
// SECTION 7: 2D Distribution Tests
// ============================================================================

TEST_F(DiracMixtureDistribution2DTest, ComputeMean) {
  std::vector<double> mean_data(2, 0.0);
  beluga_estimation::std_future::span<double> mean(mean_data.data(), 2);
  dist_.ComputeMean(mean);
  EXPECT_NEAR(mean[0], 1.0, 1e-9);
  EXPECT_NEAR(mean[1], 0.5, 1e-9);
}

TEST_F(DiracMixtureDistribution2DTest, ComputeCovariance) {
  std::vector<double> cov_data(4, 0.0);
  beluga_estimation::std_future::mdspan<double, 2> cov(cov_data.data(), 2, 2);
  dist_.ComputeCovariance(cov);
  EXPECT_NEAR(cov(0, 0), 1.00, 1e-9);
  EXPECT_NEAR(cov(1, 1), 0.25, 1e-9);
  EXPECT_NEAR(cov(0, 1), -0.50, 1e-9);
  EXPECT_NEAR(cov(1, 0), -0.50, 1e-9);
}

TEST_F(DiracMixtureDistribution2DTest, SamplePointsAreValid) {
  auto samples = dist_.Sample(rng_) | ranges::views::take(100) | ranges::to<std::vector>();
  for (const auto& s : samples) {
    EXPECT_TRUE(s.isApprox(pts_[0]) || s.isApprox(pts_[1]));
  }
}

TEST(DiracMixtureDistribution2DSampleTest, SampleStatisticalFrequencies) {
  // Skewed weights: p = [0.25, 0.75].
  const std::vector<Vec2> pts = {Vec2(0.0, 1.0), Vec2(2.0, 0.0)};
  beluga_estimation::DiracMixtureNDistribution<2> d{pts, {1.0, 3.0}};
  beluga_estimation::RandomNumberGenerator rng(42);

  constexpr std::size_t kDraws = 100000;
  std::size_t count_first = 0;
  for (const auto& s : d.Sample(rng) | ranges::views::take(kDraws)) {
    if (s.isApprox(pts[0])) {
      ++count_first;
    }
  }
  EXPECT_NEAR(static_cast<double>(count_first) / kDraws, 0.25, 0.01);
}

// ---------------------------------------------------------------------------
// Marginalize tests
// ---------------------------------------------------------------------------

TEST_F(DiracMixtureDistributionTest, MarginalizeSingleDimension) {
  // dist_ is 1D with points [0, 1, 2], weights [1, 2, 1].
  // Marginalizing dimension 0 should give the same distribution.
  auto marginalized = dist_.Marginalize(beluga_estimation::contiguous_slice::Span(0, 1));
  ASSERT_NE(marginalized, nullptr);

  beluga_estimation::Vector<double, 1> marg_mean;
  marginalized->ComputeMean(beluga_estimation::to_view(marg_mean));
  EXPECT_NEAR(marg_mean[0], 1.0, kTolerance);
}

TEST_F(DiracMixtureDistribution2DTest, MarginalizeSingleDimension) {
  // pts_ are [(0,1), (2,0)], weights [1, 1].
  // Marginalize to dimension 0 only → points [0, 2], mean = 1.0.
  auto marginalized = dist_.Marginalize(beluga_estimation::contiguous_slice::Span(0, 1));
  ASSERT_NE(marginalized, nullptr);

  beluga_estimation::Vector<double, 1> marg_mean;
  marginalized->ComputeMean(beluga_estimation::to_view(marg_mean));
  EXPECT_NEAR(marg_mean[0], 1.0, kTolerance);
}

TEST_F(DiracMixtureDistribution2DTest, MarginalizeContiguousBlock) {
  // Marginalize to both dimensions → same distribution.
  auto marginalized = dist_.Marginalize(beluga_estimation::contiguous_slice::Span(0, 2));
  ASSERT_NE(marginalized, nullptr);

  Vec2 marg_mean;
  marginalized->ComputeMean(beluga_estimation::to_view(marg_mean));
  EXPECT_TRUE(marg_mean.isApprox(Vec2(1.0, 0.5), kTolerance));
}

TEST(DiracMixtureDynamicTest, MarginalizeEmptyReturnsNullptr) {
  std::vector<Eigen::VectorXd> pts = {Eigen::VectorXd::Constant(1, 42.0)};
  beluga_estimation::DiracMixtureNDistribution<-1> dist(pts, std::vector<double>{1.0});

  EXPECT_EQ(dist.Marginalize(beluga_estimation::view<const beluga_estimation::contiguous_slice>{}), nullptr);
}

}  // namespace

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
#include <memory>
#include <random>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/distributions/se2_tangent_distribution.hpp>
#include <beluga_estimation/manifolds/se2_manifold.hpp>
#include "beluga_estimation/distribution_test_utils.hpp"

namespace beluga_estimation {
namespace {

using testing::kNumSamples;
using testing::kStatTolerance;
using testing::kTolerance;

// ============================================================================
// Fixture: identity covariance at origin, zero tangent mean.
// ============================================================================
class SE2TangentDistributionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    origin_ = beluga_estimation::SE2d();
    tangent_mean_ = beluga_estimation::Vector3d::Zero();
    covariance_ = beluga_estimation::Matrix3d::Identity();
    normal_ = std::make_shared<NormalDistribution_<double, 3>>(tangent_mean_, covariance_);
    dist_ = std::make_shared<SE2TangentDistribution_<double>>(normal_, origin_);
  }

  beluga_estimation::SE2d origin_;
  beluga_estimation::Vector3d tangent_mean_;
  beluga_estimation::Matrix3d covariance_;
  std::shared_ptr<NormalDistribution_<double, 3>> normal_;
  std::shared_ptr<SE2TangentDistribution_<double>> dist_;
};

// ============================================================================
// SECTION 1: Construction and Accessors
// ============================================================================

TEST_F(SE2TangentDistributionTest, ManifoldAndEmbeddedAccess) {
  EXPECT_EQ(dist_->GetManifold()->AmbientSize(), 4u);
  EXPECT_EQ(dist_->GetManifold()->TangentSize(), 3u);
  EXPECT_EQ(dist_->GetEmbeddedDistribution()->DomainSize(), 3u);
}

TEST_F(SE2TangentDistributionTest, MeanAtOrigin) {
  // Zero tangent mean → ambient mean = origin.
  beluga_estimation::SE2d mean;
  dist_->ComputeMean(mean);
  for (int i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(mean.data()[i], origin_.data()[i]);
  }
}

TEST_F(SE2TangentDistributionTest, MeanWithNonZeroTangent) {
  // ambient_mean = origin ⊕ tangent_mean
  beluga_estimation::Vector3d tangent_mean(1.0, 0.0, 0.0);
  auto normal = std::make_shared<NormalDistribution_<double, 3>>(tangent_mean, beluga_estimation::Matrix3d::Identity());
  SE2TangentDistribution_<double> dist(normal, origin_);

  beluga_estimation::SE2d mean;
  dist.ComputeMean(mean);

  // Expected: origin * exp(tangent_mean)
  beluga_estimation::SE2d expected_mean = origin_ * beluga_estimation::SE2d::exp(tangent_mean);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(mean.data()[i], expected_mean.data()[i], kTolerance);
  }
}

TEST_F(SE2TangentDistributionTest, CovarianceMatchesEmbedded) {
  beluga_estimation::Matrix3d cov;
  cov << 0.1, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.3;

  auto normal = std::make_shared<NormalDistribution_<double, 3>>(beluga_estimation::Vector3d::Zero(), cov);
  SE2TangentDistribution_<double> dist(normal, origin_);

  beluga_estimation::Matrix3d computed_cov;
  dist.ComputeCovariance(computed_cov);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(computed_cov(i, j), cov(i, j), kTolerance);
    }
  }
}

TEST_F(SE2TangentDistributionTest, LogProbabilityAtOrigin) {
  // At origin, tangent error = 0 → log-PDF = -0.5 * (N * log(2π))
  double expected = -0.5 * 3.0 * std::log(2.0 * M_PI);
  EXPECT_NEAR(dist_->LogProbability(origin_), expected, kTolerance);
}

TEST_F(SE2TangentDistributionTest, LogProbabilityAtKnownOffset) {
  // Displace origin by [0.5, 0, 0] in tangent space.
  beluga_estimation::SE2d displaced =
      origin_ * beluga_estimation::SE2d::exp(beluga_estimation::Vector3d(0.5, 0.0, 0.0));

  // Tangent error = (origin^{-1} * displaced).log() = [0.5, 0, 0]
  // Log-PDF = -0.5 * (N*log(2π) + error^T * cov^{-1} * error)
  // With identity covariance: = -0.5 * (3*log(2π) + 0.25)
  double expected = -0.5 * (3.0 * std::log(2.0 * M_PI) + 0.25);
  EXPECT_NEAR(dist_->LogProbability(displaced), expected, kTolerance);
}

TEST_F(SE2TangentDistributionTest, SamplesAreValidSE2) {
  RandomNumberGenerator rng(42);
  auto samples = dist_->Sample(10, rng);
  ASSERT_EQ(samples.size(), 10u);

  for (const auto& sample : samples) {
    const double* data = sample.data();
    EXPECT_NEAR(data[0] * data[0] + data[1] * data[1], 1.0, kTolerance);
  }
}

TEST_F(SE2TangentDistributionTest, SampleMatchesDistributionParameters) {
  // Use a non-diagonal covariance so cross-term accuracy is visible.
  beluga_estimation::Vector3d tangent_mean(0.5, -0.3, 0.1);
  beluga_estimation::Matrix3d cov;
  cov << 0.25, 0.05, 0.0, 0.05, 0.16, 0.0, 0.0, 0.0, 0.09;

  auto normal = std::make_shared<NormalDistribution_<double, 3>>(tangent_mean, cov);
  SE2TangentDistribution_<double> dist(normal, origin_);

  RandomNumberGenerator rng(42);
  auto samples = dist.Sample(kNumSamples, rng);

  // Map samples back to tangent space for empirical stats.
  std::vector<beluga_estimation::Vector3d> tangent_samples(kNumSamples);
  for (std::size_t i = 0; i < kNumSamples; ++i) {
    tangent_samples[i] = (origin_.inverse() * samples[i]).log();
  }

  auto [empirical_mean, empirical_cov] = testing::ComputeEmpiricalStats(tangent_samples);

  EXPECT_TRUE(empirical_mean.isApprox(tangent_mean, kStatTolerance));
  EXPECT_TRUE(empirical_cov.isApprox(cov, kStatTolerance));
}

}  // namespace
}  // namespace beluga_estimation

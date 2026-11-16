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
#include <cstddef>
#include <memory>
#include <random>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "beluga_estimation/distribution_test_utils.hpp"
#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/distribution.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/distributions/generic_tangent_distribution.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/manifolds/euclidean_manifold.hpp>
#include <beluga_estimation/manifolds/se2_manifold.hpp>

namespace beluga_estimation {
namespace {

using testing::kTolerance;

// ============================================================================
// SECTION 1: Construction and Accessors
// ============================================================================

TEST(GenericTangentDistribution, GetOrigin) {
  auto manifold = EuclideanManifold_<double, 2>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector2d(0.0, 0.0), beluga_estimation::Matrix2d::Identity());
  beluga_estimation::VectorXd origin(2);
  origin << 1.0, 2.0;
  GenericTangentDistribution_<double> dist(manifold, normal, origin);

  EXPECT_EQ(dist.GetManifold()->AmbientSize(), 2u);
  EXPECT_EQ(dist.GetManifold()->TangentSize(), 2u);
  EXPECT_EQ(dist.GetEmbeddedDistribution()->DomainSize(), 2u);

  // Origin() accessor.
  EXPECT_DOUBLE_EQ(dist.Origin()(0), 1.0);
  EXPECT_DOUBLE_EQ(dist.Origin()(1), 2.0);

  // GetOrigin(span) copies correctly.
  beluga_estimation::Vector2d retrieved;
  dist.GetOrigin(retrieved);
  EXPECT_DOUBLE_EQ(retrieved(0), 1.0);
  EXPECT_DOUBLE_EQ(retrieved(1), 2.0);
}

TEST(GenericTangentDistribution, TangentSizeViaInnerDistribution) {
  auto manifold = EuclideanManifold_<double, 3>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector3d(0.0, 0.0, 0.0), beluga_estimation::Matrix3d::Identity());
  GenericTangentDistribution_<double> dist(manifold, normal);
  EXPECT_EQ(dist.GetEmbeddedDistribution()->DomainSize(), 3u);
}

// ============================================================================
// SECTION 2: Interface Consistency (Virtual / Span vs Typed)
// ============================================================================

TEST(GenericTangentDistribution, ComputeMeanViaVirtualInterface) {
  auto manifold = EuclideanManifold_<double, 2>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector2d(3.0, 4.0), beluga_estimation::Matrix2d::Identity());
  beluga_estimation::VectorXd origin(2);
  origin << 1.0, 2.0;
  GenericTangentDistribution_<double> dist(manifold, normal, origin);

  // Compute mean via the virtual Distribution_ interface with a span.
  std::vector<double> mean_span(2, 0.0);
  dist.ComputeMean(mean_span);

  // For Euclidean manifold: mean = origin + embedded_mean = [4, 6].
  EXPECT_NEAR(mean_span[0], 4.0, kTolerance);
  EXPECT_NEAR(mean_span[1], 6.0, kTolerance);
}

TEST(GenericTangentDistribution, ComputeCovarianceViaVirtualInterface) {
  auto manifold = EuclideanManifold_<double, 2>::Instance();
  beluga_estimation::Matrix2d cov;
  cov << 1.5, 0.3, 0.3, 2.0;
  auto normal = std::make_shared<NormalDistribution_<double>>(beluga_estimation::Vector2d::Zero(), cov);
  GenericTangentDistribution_<double> dist(manifold, normal);

  // Covariance delegates directly to embedded.
  std::vector<double> cov_data(4, 0.0);
  auto cov_span = std_future::mdspan<double, 2>(cov_data.data(), 2, 2);
  const Distribution_<double>* base = &dist;
  base->ComputeCovariance(cov_span);

  EXPECT_NEAR(cov_span(0, 0), 1.5, kTolerance);
  EXPECT_NEAR(cov_span(0, 1), 0.3, kTolerance);
  EXPECT_NEAR(cov_span(1, 0), 0.3, kTolerance);
  EXPECT_NEAR(cov_span(1, 1), 2.0, kTolerance);
}

TEST(GenericTangentDistribution, LogProbabilityViaVirtualInterface) {
  // With origin=0, embedded_mean=0, identity cov:
  // LogProbability(x) = -0.5 * (N*log(2π) + ||x||^2)
  auto manifold = EuclideanManifold_<double, 2>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector2d::Zero(), beluga_estimation::Matrix2d::Identity());
  GenericTangentDistribution_<double> dist(manifold, normal);

  std::vector<double> value{1.0, 0.5};
  double log_prob = dist.LogProbability(std_future::span<const double>(value.data(), 2));

  // Expected: -0.5 * (2*log(2π) + 1² + 0.5²)
  double expected = -0.5 * (2.0 * std::log(2.0 * M_PI) + 1.25);
  EXPECT_NEAR(log_prob, expected, kTolerance);
}

TEST(GenericTangentDistribution, SampleViaVirtualInterface) {
  auto manifold = EuclideanManifold_<double, 2>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector2d(1.0, 2.0), beluga_estimation::Matrix2d::Identity());
  GenericTangentDistribution_<double> dist(manifold, normal);

  // Sample via virtual interface.
  RandomNumberGenerator rng(42);
  std::vector<std::vector<double>> buffers(5, std::vector<double>(2, 0.0));
  std::vector<std_future::span<double>> spans;
  for (auto& buf : buffers) {
    spans.emplace_back(buf.data(), 2);
  }
  const Distribution_<double>* base = &dist;
  base->Sample(beluga_estimation::to_view(spans), rng);

  // Verify all samples have the correct dimension.
  for (const auto& buf : buffers) {
    EXPECT_EQ(buf.size(), 2u);
  }
}

// ============================================================================
// SECTION 3: Marginalization
// ============================================================================

TEST(GenericTangentDistribution, MarginalizeFullSet) {
  auto manifold = EuclideanManifold_<double, 3>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector3d(1.0, 2.0, 3.0), beluga_estimation::Matrix3d::Identity());
  GenericTangentDistribution_<double> dist(manifold, normal);

  beluga_estimation::contiguous_slice dims[]{beluga_estimation::contiguous_slice::Span(0, 3)};
  auto result = dist.Marginalize(dims);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->GetEmbeddedDistribution()->DomainSize(), 3u);
  EXPECT_NE(result->GetManifold(), nullptr);
}

TEST(GenericTangentDistribution, MarginalizeSingleDimension) {
  auto manifold = EuclideanManifold_<double, 3>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector3d(1.0, 2.0, 3.0), beluga_estimation::Matrix3d::Identity());
  GenericTangentDistribution_<double> dist(manifold, normal);

  beluga_estimation::contiguous_slice dims[]{beluga_estimation::contiguous_slice::Span(1, 1)};
  auto result = dist.Marginalize(dims);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->GetEmbeddedDistribution()->DomainSize(), 1u);

  // Verify the marginalized embedded mean (should be 2.0 from dim 1).
  beluga_estimation::VectorXd marg_mean(1);
  result->GetEmbeddedDistribution()->ComputeMean(beluga_estimation::to_view(marg_mean));
  EXPECT_NEAR(marg_mean[0], 2.0, kTolerance);
}

TEST(GenericTangentDistribution, MarginalizeMultipleSlices) {
  auto manifold = EuclideanManifold_<double, 3>::Instance();
  beluga_estimation::Matrix3d cov;
  cov << 1.0, 0.2, 0.1, 0.2, 2.0, 0.3, 0.1, 0.3, 3.0;

  auto normal = std::make_shared<NormalDistribution_<double>>(beluga_estimation::Vector3d(1.0, 2.0, 3.0), cov);
  GenericTangentDistribution_<double> dist(manifold, normal);

  // Retain dimensions 0 and 2 (skip dim 1).
  beluga_estimation::contiguous_slice dims[]{
      beluga_estimation::contiguous_slice::Span(0, 1), beluga_estimation::contiguous_slice::Span(2, 1)};
  auto result = dist.Marginalize(dims);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->GetEmbeddedDistribution()->DomainSize(), 2u);

  // Check marginalized embedded mean: [1.0, 3.0].
  beluga_estimation::VectorXd marg_mean(2);
  result->GetEmbeddedDistribution()->ComputeMean(beluga_estimation::to_view(marg_mean));
  EXPECT_NEAR(marg_mean[0], 1.0, kTolerance);
  EXPECT_NEAR(marg_mean[1], 3.0, kTolerance);

  // Check marginalized covariance blocks.
  beluga_estimation::MatrixXd marg_cov(2, 2);
  marg_cov.setZero();
  result->GetEmbeddedDistribution()->ComputeCovariance(beluga_estimation::to_view(marg_cov));
  EXPECT_NEAR(marg_cov(0, 0), 1.0, kTolerance);  // cov[0][0]
  EXPECT_NEAR(marg_cov(0, 1), 0.1, kTolerance);  // cov[0][2]
  EXPECT_NEAR(marg_cov(1, 0), 0.1, kTolerance);  // cov[2][0]
  EXPECT_NEAR(marg_cov(1, 1), 3.0, kTolerance);  // cov[2][2]
}

TEST(GenericTangentDistribution, MarginalizeContiguousBlock) {
  auto manifold = EuclideanManifold_<double, 2>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector2d(5.0, 6.0), beluga_estimation::Matrix2d::Identity());
  GenericTangentDistribution_<double> dist(manifold, normal);

  // Retain both dimensions.
  beluga_estimation::contiguous_slice dims[]{beluga_estimation::contiguous_slice::Span(0, 2)};
  auto result = dist.Marginalize(dims);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->GetEmbeddedDistribution()->DomainSize(), 2u);

  // Mean should be unchanged: [5.0, 6.0].
  beluga_estimation::VectorXd marg_mean(2);
  result->GetEmbeddedDistribution()->ComputeMean(beluga_estimation::to_view(marg_mean));
  EXPECT_NEAR(marg_mean[0], 5.0, kTolerance);
  EXPECT_NEAR(marg_mean[1], 6.0, kTolerance);
}

TEST(GenericTangentDistribution, MarginalizeOutOfRangeReturnsNull) {
  auto manifold = EuclideanManifold_<double, 2>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector2d(0.0, 0.0), beluga_estimation::Matrix2d::Identity());
  GenericTangentDistribution_<double> dist(manifold, normal);

  beluga_estimation::contiguous_slice dims[]{beluga_estimation::contiguous_slice::Span(5, 1)};  // out of range
  auto result = dist.Marginalize(dims);
  EXPECT_EQ(result, nullptr);
}

TEST(GenericTangentDistribution, MarginalizeEmptyReturnsNull) {
  auto manifold = EuclideanManifold_<double, 2>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector2d(0.0, 0.0), beluga_estimation::Matrix2d::Identity());
  GenericTangentDistribution_<double> dist(manifold, normal);

  auto result = dist.Marginalize(beluga_estimation::view<const beluga_estimation::contiguous_slice>{});
  EXPECT_EQ(result, nullptr);
}

TEST(GenericTangentDistribution, MarginalizeOverlappingSlicesReturnsNull) {
  auto manifold = EuclideanManifold_<double, 3>::Instance();
  auto normal = std::make_shared<NormalDistribution_<double>>(
      beluga_estimation::Vector3d(0.0, 0.0, 0.0), beluga_estimation::Matrix3d::Identity());
  GenericTangentDistribution_<double> dist(manifold, normal);

  // Overlap at dimension 1.
  beluga_estimation::contiguous_slice dims[]{
      beluga_estimation::contiguous_slice::Span(0, 2), beluga_estimation::contiguous_slice::Span(1, 1)};
  auto result = dist.Marginalize(dims);
  EXPECT_EQ(result, nullptr);
}

}  // namespace
}  // namespace beluga_estimation

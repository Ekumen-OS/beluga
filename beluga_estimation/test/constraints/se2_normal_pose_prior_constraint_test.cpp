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
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/constraints/se2_normal_pose_prior_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/distributions/se2_normal_distribution.hpp>

namespace {

TEST(SE2NormalPosePriorConstraintTest, DependentAndConditioningVariables) {
  beluga_estimation::Identifier id{"test_prior_constraint"};
  beluga_estimation::Identifier target_id{"pose_a"};

  beluga_estimation::SE2d prior_mean(beluga_estimation::SO2d(M_PI / 4.0), beluga_estimation::Vector2d(1.0, 2.0));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> prior(prior_mean, covariance);

  beluga_estimation::SE2NormalPosePriorConstraint_<double> constraint(id, target_id, prior);

  EXPECT_TRUE(constraint.HasDependentVariables());
  EXPECT_FALSE(constraint.HasConditioningVariables());
  EXPECT_EQ(constraint.DependentVariables().size(), 1);
  EXPECT_EQ(constraint.DependentVariables()[0].name, "pose_a");
  EXPECT_EQ(constraint.ConditioningVariables().size(), 0);
}

TEST(SE2NormalPosePriorConstraintTest, ComputeLogLikelihood) {
  beluga_estimation::Identifier id{"test_prior_constraint"};
  beluga_estimation::Identifier target_id{"pose_a"};

  beluga_estimation::SE2d prior_mean(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(1.0, 1.0));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> prior(prior_mean, covariance);

  beluga_estimation::SE2NormalPosePriorConstraint_<double> constraint(id, target_id, prior);

  beluga_estimation::SE2d target_pose_perfect(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(1.0, 1.0));
  beluga_estimation::SE2d target_pose_offset(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(2.0, 1.0));

  std::vector<beluga_estimation::std_future::span<double>> perfect_dependent_values = {{target_pose_perfect.data(), 4}};
  std::vector<beluga_estimation::std_future::span<double>> offset_dependent_values = {{target_pose_offset.data(), 4}};
  std::vector<beluga_estimation::std_future::span<double>> empty_conditioning_values;

  double log_likelihood_perfect = constraint.ComputeLogLikelihood(perfect_dependent_values, empty_conditioning_values);
  double log_likelihood_offset = constraint.ComputeLogLikelihood(offset_dependent_values, empty_conditioning_values);

  // Normalized log likelihood for 3D Gaussian with identity covariance
  // Normalization constant: -0.5 * 3 * log(2π) ≈ -2.7568
  constexpr double kLog2Pi = std::log(2.0 * M_PI);
  constexpr double kNormalizationConstant = -0.5 * 3.0 * kLog2Pi;

  // Perfect match: Mahalanobis distance = 0
  EXPECT_NEAR(log_likelihood_perfect, kNormalizationConstant, 1e-6);

  // Offset of 1.0 along X with Identity covariance yields Mahalanobis dist squared = 1.0.
  // Therefore, the normalized log likelihood is -0.5 * 1.0 + normalization_constant
  EXPECT_NEAR(log_likelihood_offset, -0.5 + kNormalizationConstant, 1e-6);
}

TEST(SE2NormalPosePriorConstraintTest, TakeSamples) {
  beluga_estimation::Identifier id{"test_prior_constraint"};
  beluga_estimation::Identifier target_id{"pose_a"};

  beluga_estimation::SE2d prior_mean(beluga_estimation::SO2d(M_PI / 2.0), beluga_estimation::Vector2d(3.0, -1.0));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity() * 0.01;
  beluga_estimation::SE2NormalDistribution_<double> prior(prior_mean, covariance);

  beluga_estimation::SE2NormalPosePriorConstraint_<double> constraint(id, target_id, prior);

  // P=0 conditioning variables (empty), N=1 samples of M=1 dependent variables
  std::vector<beluga_estimation::SE2d> empty_conditioning;
  std::vector<std::vector<beluga_estimation::SE2d>> sampled_dependent = {{beluga_estimation::SE2d{}}};

  beluga_estimation::RandomNumberGenerator generator(42);
  constraint.TakeSamples(
      beluga_estimation::to_view(empty_conditioning), beluga_estimation::to_view(sampled_dependent), generator);

  auto error = (prior_mean.inverse() * sampled_dependent[0][0]).log();

  // The error vector norm should be relatively small since the variance is 0.01 (sigma=0.1)
  EXPECT_NEAR(error.norm(), 0.0, 0.5);
}

TEST(SE2NormalPosePriorConstraintTest, CalcMean) {
  beluga_estimation::Identifier id{"test_prior_constraint"};
  beluga_estimation::Identifier target_id{"pose_a"};

  beluga_estimation::SE2d prior_mean(beluga_estimation::SO2d(M_PI / 4.0), beluga_estimation::Vector2d(1.0, -1.0));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> prior(prior_mean, covariance);

  beluga_estimation::SE2NormalPosePriorConstraint_<double> constraint(id, target_id, prior);

  std::vector<beluga_estimation::std_future::span<double>> empty_conditioning_values;

  beluga_estimation::SE2d actual_mean;
  beluga_estimation::std_future::span<double> mean_span(actual_mean.data(), 4);

  constraint.CalcMean(empty_conditioning_values, mean_span);

  EXPECT_NEAR(actual_mean.translation().x(), prior_mean.translation().x(), 1e-6);
  EXPECT_NEAR(actual_mean.translation().y(), prior_mean.translation().y(), 1e-6);
  EXPECT_NEAR(actual_mean.so2().log(), prior_mean.so2().log(), 1e-6);

  // Verify that passing a Jacobian/Hessian pointer appropriately throws
  beluga_estimation::Matrix3d actual_jacobian;
  beluga_estimation::std_future::mdspan<double, 2> jacobian(actual_jacobian.data(), 3, 3);
  EXPECT_THROW(constraint.CalcMean(empty_conditioning_values, mean_span, jacobian), std::runtime_error);
}

TEST(SE2NormalPosePriorConstraintTest, CalcCovariance) {
  beluga_estimation::Identifier id{"test_prior_constraint"};
  beluga_estimation::Identifier target_id{"pose_a"};

  beluga_estimation::SE2d prior_mean(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(0.0, 0.0));
  beluga_estimation::Matrix3d covariance;
  covariance << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  beluga_estimation::SE2NormalDistribution_<double> prior(prior_mean, covariance);

  beluga_estimation::SE2NormalPosePriorConstraint_<double> constraint(id, target_id, prior);

  beluga_estimation::Matrix3d actual_covariance;
  beluga_estimation::std_future::mdspan<double, 2> cov_mdspan(actual_covariance.data(), 3, 3);

  std::vector<beluga_estimation::std_future::span<double>> empty_conditioning_values;
  constraint.CalcCovariance(empty_conditioning_values, cov_mdspan);

  EXPECT_TRUE(actual_covariance.isApprox(covariance, 1e-6));
}

TEST(SE2NormalPosePriorConstraintTest, CalcResidual) {
  beluga_estimation::Identifier id{"test_prior_constraint"};
  beluga_estimation::Identifier target_id{"pose_a"};

  beluga_estimation::SE2d prior_mean(beluga_estimation::SO2d(M_PI / 8.0), beluga_estimation::Vector2d(1.0, 0.5));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> prior(prior_mean, covariance);

  beluga_estimation::SE2NormalPosePriorConstraint_<double> constraint(id, target_id, prior);

  beluga_estimation::SE2d target_pose(beluga_estimation::SO2d(0.2), beluga_estimation::Vector2d(2.0, 1.5));

  std::vector<beluga_estimation::std_future::span<double>> empty_conditioning_values;
  std::vector<beluga_estimation::std_future::span<double>> dependent_values = {{target_pose.data(), 4}};

  beluga_estimation::Vector3d actual_residual;
  beluga_estimation::std_future::span<double> residual_span(actual_residual.data(), constraint.ResidualSize());

  beluga_estimation::Matrix3d actual_jacobian;
  beluga_estimation::std_future::mdspan<double, 2> jacobian(actual_jacobian.data(), 3, 3);

  constraint.CalcResidual(dependent_values, empty_conditioning_values, residual_span, jacobian);

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(actual_residual.size()));

  beluga_estimation::Vector3d expected_residual = prior.CalculateResidual(target_pose);
  EXPECT_TRUE(actual_residual.isApprox(expected_residual, 1e-6));

  // Calculate expected Jacobian using BCH formulas for Jr^-1
  beluga_estimation::Matrix3d expected_jacobian = beluga_estimation::Matrix3d::Identity();
  const double half_theta = expected_residual(2) / 2.0;
  expected_jacobian(0, 1) = -half_theta;
  expected_jacobian(1, 0) = half_theta;
  expected_jacobian(0, 2) = expected_residual(1) / 2.0;
  expected_jacobian(1, 2) = -expected_residual(0) / 2.0;

  EXPECT_TRUE(actual_jacobian.isApprox(expected_jacobian, 1e-6));

  // Verify that passing a Hessian pointer appropriately throws
  EXPECT_THROW(
      constraint.CalcResidual(dependent_values, empty_conditioning_values, residual_span, std::nullopt, jacobian),
      std::runtime_error);
}

// ======================================================
// Section 1: Interface Consistency Tests
// ======================================================

TEST(SE2NormalPosePriorConstraintTest, CalcMeanTypedVsSpan) {
  beluga_estimation::Identifier id{"test_prior_constraint"};
  beluga_estimation::Identifier target_id{"pose_a"};

  beluga_estimation::SE2d prior_mean(beluga_estimation::SO2d(M_PI / 4.0), beluga_estimation::Vector2d(1.0, -1.0));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> prior(prior_mean, covariance);

  beluga_estimation::SE2NormalPosePriorConstraint_<double> constraint(id, target_id, prior);

  // Call span-based interface
  std::vector<beluga_estimation::std_future::span<double>> empty_conditioning_values;
  beluga_estimation::SE2d mean_span;
  beluga_estimation::std_future::span<double> mean_span_view(mean_span.data(), 4);

  constraint.CalcMean(empty_conditioning_values, mean_span_view);

  // Call typed interface
  beluga_estimation::SE2d mean_typed;
  constraint.CalcMean(mean_typed);

  // Compare results
  EXPECT_NEAR(mean_span.translation().x(), mean_typed.translation().x(), 1e-10);
  EXPECT_NEAR(mean_span.translation().y(), mean_typed.translation().y(), 1e-10);
  EXPECT_NEAR(mean_span.so2().log(), mean_typed.so2().log(), 1e-10);
}

TEST(SE2NormalPosePriorConstraintTest, CalcCovarianceTypedVsSpan) {
  beluga_estimation::Identifier id{"test_prior_constraint"};
  beluga_estimation::Identifier target_id{"pose_a"};

  beluga_estimation::SE2d prior_mean(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(0.0, 0.0));
  beluga_estimation::Matrix3d covariance;
  covariance << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  beluga_estimation::SE2NormalDistribution_<double> prior(prior_mean, covariance);

  beluga_estimation::SE2NormalPosePriorConstraint_<double> constraint(id, target_id, prior);

  // Call span-based interface
  std::vector<beluga_estimation::std_future::span<double>> empty_conditioning_values;
  beluga_estimation::Matrix3d covariance_span;
  beluga_estimation::std_future::mdspan<double, 2> cov_mdspan_span(covariance_span.data(), 3, 3);
  constraint.CalcCovariance(empty_conditioning_values, cov_mdspan_span);

  // Call typed interface
  beluga_estimation::Matrix3d covariance_typed;
  constraint.CalcCovariance(covariance_typed);

  // Compare results
  EXPECT_TRUE(covariance_span.isApprox(covariance_typed, 1e-10));
}

TEST(SE2NormalPosePriorConstraintTest, CalcResidualTypedVsSpan) {
  beluga_estimation::Identifier id{"test_prior_constraint"};
  beluga_estimation::Identifier target_id{"pose_a"};

  beluga_estimation::SE2d prior_mean(beluga_estimation::SO2d(M_PI / 8.0), beluga_estimation::Vector2d(1.0, 0.5));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> prior(prior_mean, covariance);

  beluga_estimation::SE2NormalPosePriorConstraint_<double> constraint(id, target_id, prior);

  beluga_estimation::SE2d target_pose(beluga_estimation::SO2d(0.2), beluga_estimation::Vector2d(2.0, 1.5));

  // Call span-based interface
  std::vector<beluga_estimation::std_future::span<double>> empty_conditioning_values;
  std::vector<beluga_estimation::std_future::span<double>> dependent_values_span = {{target_pose.data(), 4}};
  beluga_estimation::Vector3d residual_span;
  beluga_estimation::std_future::span<double> residual_span_view(residual_span.data(), constraint.ResidualSize());
  beluga_estimation::Matrix3d jacobian_span;
  beluga_estimation::std_future::mdspan<double, 2> jacobian_mdspan_span(jacobian_span.data(), 3, 3);

  constraint.CalcResidual(dependent_values_span, empty_conditioning_values, residual_span_view, jacobian_mdspan_span);

  // Call typed interface
  beluga_estimation::Vector3d residual_typed;
  beluga_estimation::Matrix3d jacobian_typed;
  constraint.CalcResidual(target_pose, residual_typed, jacobian_typed);

  // Compare results
  EXPECT_TRUE(residual_span.isApprox(residual_typed, 1e-10));
  EXPECT_TRUE(jacobian_span.isApprox(jacobian_typed, 1e-10));
}

}  // namespace

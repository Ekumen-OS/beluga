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
#include <beluga_estimation/constraints/se2_normal_pose_likelihood_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/distributions/se2_normal_distribution.hpp>

namespace {

TEST(SE2NormalPoseLikelihoodConstraintTest, DependentAndConditioningVariables) {
  beluga_estimation::Identifier id{"test_likelihood"};
  beluga_estimation::Identifier conditioning_id{"pose_x"};

  beluga_estimation::SE2d z_obs(beluga_estimation::SO2d(M_PI / 4.0), beluga_estimation::Vector2d(1.0, 2.0));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> measurement(z_obs, covariance);

  beluga_estimation::SE2NormalPoseLikelihoodConstraint_<double> constraint(id, measurement, conditioning_id);

  EXPECT_FALSE(constraint.HasDependentVariables());
  EXPECT_TRUE(constraint.HasConditioningVariables());
  EXPECT_EQ(constraint.DependentVariables().size(), 0);
  EXPECT_EQ(constraint.ConditioningVariables().size(), 1);
  EXPECT_EQ(constraint.ConditioningVariables()[0].name, "pose_x");
}

TEST(SE2NormalPoseLikelihoodConstraintTest, ComputeLogLikelihood) {
  beluga_estimation::Identifier id{"test_likelihood"};
  beluga_estimation::Identifier conditioning_id{"pose_x"};

  beluga_estimation::SE2d z_obs(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(1.0, 1.0));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> measurement(z_obs, covariance);

  beluga_estimation::SE2NormalPoseLikelihoodConstraint_<double> constraint(id, measurement, conditioning_id);

  beluga_estimation::SE2d x_perfect(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(1.0, 1.0));
  beluga_estimation::SE2d x_offset(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(2.0, 1.0));

  std::vector<beluga_estimation::std_future::span<double>> empty_dependent_values;
  std::vector<beluga_estimation::std_future::span<double>> perfect_conditioning = {{x_perfect.data(), 4}};
  std::vector<beluga_estimation::std_future::span<double>> offset_conditioning = {{x_offset.data(), 4}};

  double log_likelihood_perfect = constraint.ComputeLogLikelihood(empty_dependent_values, perfect_conditioning);
  double log_likelihood_offset = constraint.ComputeLogLikelihood(empty_dependent_values, offset_conditioning);

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

TEST(SE2NormalPoseLikelihoodConstraintTest, TakeSamplesThrows) {
  beluga_estimation::Identifier id{"test_likelihood"};
  beluga_estimation::Identifier conditioning_id{"pose_x"};

  beluga_estimation::SE2d z_obs(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(0.0, 0.0));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> measurement(z_obs, covariance);

  beluga_estimation::SE2NormalPoseLikelihoodConstraint_<double> constraint(id, measurement, conditioning_id);

  std::vector<beluga_estimation::SE2d> conditioning = {
      beluga_estimation::SE2d(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(0.0, 0.0))};
  std::vector<std::vector<beluga_estimation::SE2d>> sampled_dependent = {{beluga_estimation::SE2d{}}};

  beluga_estimation::RandomNumberGenerator generator(42);
  EXPECT_THROW(
      constraint.TakeSamples(
          beluga_estimation::to_view(conditioning), beluga_estimation::to_view(sampled_dependent), generator),
      std::runtime_error);
}

TEST(SE2NormalPoseLikelihoodConstraintTest, CalcMeanThrows) {
  beluga_estimation::Identifier id{"test_likelihood"};
  beluga_estimation::Identifier conditioning_id{"pose_x"};

  beluga_estimation::SE2d z_obs(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(0.0, 0.0));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> measurement(z_obs, covariance);

  beluga_estimation::SE2NormalPoseLikelihoodConstraint_<double> constraint(id, measurement, conditioning_id);

  beluga_estimation::SE2d x(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(1.0, 2.0));
  std::vector<beluga_estimation::std_future::span<double>> conditioning_values = {{x.data(), 4}};

  beluga_estimation::SE2d mean;
  beluga_estimation::std_future::span<double> mean_span(mean.data(), 4);

  EXPECT_THROW(constraint.CalcMean(conditioning_values, mean_span), std::runtime_error);
}

TEST(SE2NormalPoseLikelihoodConstraintTest, CalcCovariance) {
  beluga_estimation::Identifier id{"test_likelihood"};
  beluga_estimation::Identifier conditioning_id{"pose_x"};

  beluga_estimation::SE2d z_obs(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(0.0, 0.0));
  beluga_estimation::Matrix3d covariance;
  covariance << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  beluga_estimation::SE2NormalDistribution_<double> measurement(z_obs, covariance);

  beluga_estimation::SE2NormalPoseLikelihoodConstraint_<double> constraint(id, measurement, conditioning_id);

  beluga_estimation::Matrix3d actual_covariance;
  beluga_estimation::std_future::mdspan<double, 2> cov_mdspan(actual_covariance.data(), 3, 3);

  beluga_estimation::SE2d x(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(1.0, 2.0));
  std::vector<beluga_estimation::std_future::span<double>> conditioning_values = {{x.data(), 4}};
  constraint.CalcCovariance(conditioning_values, cov_mdspan);

  EXPECT_TRUE(actual_covariance.isApprox(covariance, 1e-6));
}

TEST(SE2NormalPoseLikelihoodConstraintTest, CalcResidual) {
  beluga_estimation::Identifier id{"test_likelihood"};
  beluga_estimation::Identifier conditioning_id{"pose_x"};

  beluga_estimation::SE2d z_obs(beluga_estimation::SO2d(M_PI / 8.0), beluga_estimation::Vector2d(1.0, 0.5));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> measurement(z_obs, covariance);

  beluga_estimation::SE2NormalPoseLikelihoodConstraint_<double> constraint(id, measurement, conditioning_id);

  beluga_estimation::SE2d x(beluga_estimation::SO2d(0.2), beluga_estimation::Vector2d(2.0, 1.5));

  std::vector<beluga_estimation::std_future::span<double>> empty_dependent_values;
  std::vector<beluga_estimation::std_future::span<double>> conditioning_values = {{x.data(), 4}};

  beluga_estimation::Vector3d actual_residual;
  beluga_estimation::std_future::span<double> residual_span(actual_residual.data(), constraint.ResidualSize());

  beluga_estimation::Matrix3d actual_jacobian;
  beluga_estimation::std_future::mdspan<double, 2> jacobian(actual_jacobian.data(), 3, 3);

  constraint.CalcResidual(empty_dependent_values, conditioning_values, residual_span, jacobian);

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(actual_residual.size()));

  // Residual = log(x^-1 * z_obs) = -log(z_obs^-1 * x).
  beluga_estimation::Vector3d expected_residual = -measurement.CalculateResidual(x);
  EXPECT_TRUE(actual_residual.isApprox(expected_residual, 1e-6));

  // The EKF-consistent Jacobian H = -d/dx(log(z_obs^-1 * x)) equals
  // Jr^{-1}(-r) evaluated at the residual r = log(x^-1 * z_obs).
  beluga_estimation::Matrix3d expected_jacobian = beluga_estimation::Matrix3d::Identity();
  const double half_theta = expected_residual(2) / 2.0;
  expected_jacobian(0, 1) = half_theta;
  expected_jacobian(1, 0) = -half_theta;
  expected_jacobian(0, 2) = -expected_residual(1) / 2.0;
  expected_jacobian(1, 2) = expected_residual(0) / 2.0;

  EXPECT_TRUE(actual_jacobian.isApprox(expected_jacobian, 1e-6));
}

TEST(SE2NormalPoseLikelihoodConstraintTest, CalcResidualJacobianMatchesFiniteDifference) {
  beluga_estimation::Identifier id{"test_likelihood"};
  beluga_estimation::Identifier conditioning_id{"pose_x"};

  const beluga_estimation::SE2d z_obs(beluga_estimation::SO2d(0.3), beluga_estimation::Vector2d(1.0, 0.5));
  const beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  const beluga_estimation::SE2NormalDistribution_<double> measurement(z_obs, covariance);
  const beluga_estimation::SE2NormalPoseLikelihoodConstraint_<double> constraint(id, measurement, conditioning_id);

  // Keep the state close to the measurement so the residual is small: the
  // reported Jacobian is a first-order BCH approximation, which matches the
  // true derivative up to O(|r|^2) terms.
  const beluga_estimation::Vector3d offset(1e-3, 2e-3, -1.5e-3);
  beluga_estimation::SE2d x = z_obs * beluga_estimation::SE2d::exp(offset);

  beluga_estimation::Vector3d residual;
  beluga_estimation::Matrix3d jacobian;
  constraint.CalcResidual(x, residual, jacobian);

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  // Independent finite-difference check of the measurement model Jacobian.
  // With r(δ) = log((x exp(δ))^-1 z_obs) under a right perturbation of x, the
  // reported Jacobian is H = -dr/dδ, so each column must satisfy
  // H(:,j) ≈ -(r(δ_j) - r(0)) / ε.
  constexpr double kEps = 1e-7;
  for (int j = 0; j < 3; ++j) {
    beluga_estimation::Vector3d delta = beluga_estimation::Vector3d::Zero();
    delta(j) = kEps;
    const beluga_estimation::SE2d x_perturbed = x * beluga_estimation::SE2d::exp(delta);
    beluga_estimation::Vector3d perturbed_residual;
    constraint.CalcResidual(x_perturbed, perturbed_residual);
    const beluga_estimation::Vector3d numeric_column = -(perturbed_residual - residual) / kEps;
    EXPECT_TRUE(jacobian.col(j).isApprox(numeric_column, 1e-5)) << "column " << j;
  }
}

TEST(SE2NormalPoseLikelihoodConstraintTest, CalcResidualHessianThrows) {
  beluga_estimation::Identifier id{"test_likelihood"};
  beluga_estimation::Identifier conditioning_id{"pose_x"};

  beluga_estimation::SE2d z_obs(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(0.0, 0.0));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> measurement(z_obs, covariance);

  beluga_estimation::SE2NormalPoseLikelihoodConstraint_<double> constraint(id, measurement, conditioning_id);

  beluga_estimation::SE2d x(beluga_estimation::SO2d(0.1), beluga_estimation::Vector2d(1.0, 2.0));

  std::vector<beluga_estimation::std_future::span<double>> empty_dependent_values;
  std::vector<beluga_estimation::std_future::span<double>> conditioning_values = {{x.data(), 4}};

  beluga_estimation::Vector3d residual;
  beluga_estimation::std_future::span<double> residual_span(residual.data(), 3);

  beluga_estimation::Matrix3d jacobian;
  beluga_estimation::std_future::mdspan<double, 2> jacobian_mdspan(jacobian.data(), 3, 3);

  beluga_estimation::Matrix3d hessian;
  beluga_estimation::std_future::mdspan<double, 2> hessian_mdspan(hessian.data(), 3, 3);

  EXPECT_THROW(
      constraint.CalcResidual(
          empty_dependent_values, conditioning_values, residual_span, jacobian_mdspan, hessian_mdspan),
      std::runtime_error);
}

// ======================================================
// Section 2: Interface Consistency Tests
// ======================================================

TEST(SE2NormalPoseLikelihoodConstraintTest, CalcCovarianceTypedVsSpan) {
  beluga_estimation::Identifier id{"test_likelihood"};
  beluga_estimation::Identifier conditioning_id{"pose_x"};

  beluga_estimation::SE2d z_obs(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(0.0, 0.0));
  beluga_estimation::Matrix3d covariance;
  covariance << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  beluga_estimation::SE2NormalDistribution_<double> measurement(z_obs, covariance);

  beluga_estimation::SE2NormalPoseLikelihoodConstraint_<double> constraint(id, measurement, conditioning_id);

  // Call span-based interface
  beluga_estimation::SE2d x(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(1.0, 2.0));
  std::vector<beluga_estimation::std_future::span<double>> conditioning_values = {{x.data(), 4}};
  beluga_estimation::Matrix3d covariance_span;
  beluga_estimation::std_future::mdspan<double, 2> cov_mdspan_span(covariance_span.data(), 3, 3);
  constraint.CalcCovariance(conditioning_values, cov_mdspan_span);

  // Call typed interface
  beluga_estimation::Matrix3d covariance_typed;
  constraint.CalcCovariance(covariance_typed);

  // Compare results
  EXPECT_TRUE(covariance_span.isApprox(covariance_typed, 1e-10));
}

TEST(SE2NormalPoseLikelihoodConstraintTest, CalcResidualTypedVsSpan) {
  beluga_estimation::Identifier id{"test_likelihood"};
  beluga_estimation::Identifier conditioning_id{"pose_x"};

  beluga_estimation::SE2d z_obs(beluga_estimation::SO2d(M_PI / 8.0), beluga_estimation::Vector2d(1.0, 0.5));
  beluga_estimation::Matrix3d covariance = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::SE2NormalDistribution_<double> measurement(z_obs, covariance);

  beluga_estimation::SE2NormalPoseLikelihoodConstraint_<double> constraint(id, measurement, conditioning_id);

  beluga_estimation::SE2d x(beluga_estimation::SO2d(0.2), beluga_estimation::Vector2d(2.0, 1.5));

  // Call span-based interface
  std::vector<beluga_estimation::std_future::span<double>> empty_dependent_values;
  std::vector<beluga_estimation::std_future::span<double>> conditioning_values_span = {{x.data(), 4}};
  beluga_estimation::Vector3d residual_span;
  beluga_estimation::std_future::span<double> residual_span_view(residual_span.data(), constraint.ResidualSize());
  beluga_estimation::Matrix3d jacobian_span;
  beluga_estimation::std_future::mdspan<double, 2> jacobian_mdspan_span(jacobian_span.data(), 3, 3);

  constraint.CalcResidual(empty_dependent_values, conditioning_values_span, residual_span_view, jacobian_mdspan_span);

  // Call typed interface
  beluga_estimation::Vector3d residual_typed;
  beluga_estimation::Matrix3d jacobian_typed;
  constraint.CalcResidual(x, residual_typed, jacobian_typed);

  // Compare results
  EXPECT_TRUE(residual_span.isApprox(residual_typed, 1e-10));
  EXPECT_TRUE(jacobian_span.isApprox(jacobian_typed, 1e-10));
}

}  // namespace

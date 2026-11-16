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

#pragma once

#include <cassert>
#include <optional>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/locally_normal_constraint.hpp>
#include <beluga_estimation/distributions/se2_normal_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief SE2 likelihood constraint for observed pose measurements.
 *
 * Models a likelihood constraint p(.|x): evaluates p(z_obs|x) where z_obs is fixed.
 *
 * This constraint has only conditioning variables (no dependent variables).
 * The observed measurement z_obs is stored in the distribution's mean,
 * and the measurement uncertainty is expressed in the covariance.
 */
template <typename T>
class SE2NormalPoseLikelihoodConstraint_ : public LocallyNormalConstraint_<T> {
 public:
  SE2NormalPoseLikelihoodConstraint_(
      Identifier id,
      const SE2NormalDistribution_<T>& measurement,
      Identifier conditioning_id)
      : LocallyNormalConstraint_<T>(std::move(id)),
        conditioning_id_{std::move(conditioning_id)},
        measurement_{measurement} {}

  bool HasDependentVariables() const override { return false; }
  bool HasConditioningVariables() const override { return true; }

  std_future::span<const Identifier> DependentVariables() const override { return {}; }

  std_future::span<const Identifier> ConditioningVariables() const override { return {&conditioning_id_, 1}; }

  [[nodiscard]] std::size_t ResidualSize() const override { return beluga_estimation::SE2<T>::DoF; }

  T ComputeLogLikelihood(
      view<std_future::span<const T>> dependent_values,
      view<std_future::span<const T>> conditioning_values) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_values.size() == 0);
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 1);
    BELUGA_ESTIMATION_ASSERT(conditioning_values[0].size() == 4);  // SE(2) ambient size.

    // Evaluate p(z_obs|x) directly using the measurement distribution
    return measurement_.LogProbability(conditioning_values[0]);
  }

  void TakeSamples(
      [[maybe_unused]] view<std_future::span<const T>> conditioning_values,
      [[maybe_unused]] view<view<std_future::span<T>>> sampled_dependent_values,
      [[maybe_unused]] RandomNumberGenerator& generator) const override {
    BELUGA_ESTIMATION_DEMAND(false, "sampling is not supported for likelihood constraints");
  }

  /**
   * @brief Mean calculation is not supported for likelihood constraints.
   *
   * @throws std::runtime_error always.
   */
  void CalcMean(
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> mean,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_DEMAND(false, "mean calculation is not supported for likelihood constraints");
  }

  /**
   * @brief Provides the tangent-space covariance of the measurement (typed interface).
   *
   * @param covariance Output: The covariance matrix.
   */
  void CalcCovariance(beluga_estimation::Ref<beluga_estimation::Matrix3<T>> covariance) const {
    covariance = measurement_.Covariance();
  }

  /**
   * @brief Provides the tangent-space covariance of the measurement (span interface).
   */
  void CalcCovariance(
      [[maybe_unused]] view<std_future::span<const T>> conditioning_variables,
      std_future::mdspan<T, 2> covariance) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 1);
    auto cov_map = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(covariance);
    CalcCovariance(cov_map);
  }

  /**
   * @brief Calculates the residual and its Jacobian in the tangent space (typed interface).
   * r = log( x^-1 * z_obs )
   *
   * The residual is the innovation `measurement - h(state)` in tangent space.
   * The optional Jacobian is the EKF-consistent measurement model Jacobian
   * H = d/dx (h(x) ⊖ z_obs) evaluated at the residual; note this is the
   * negative of the residual's own derivative, consistent with the EKF update
   * `x += K * r`.
   *
   * @param x The conditioning pose (conditioning variable).
   * @param residual Output: The residual in tangent space (3x1).
   * @param jacobian Optional: Jacobian of the measurement model w.r.t. x (3x3).
   * @param hessian Optional: Hessian (not supported, throws if provided).
   */
  void CalcResidual(
      beluga_estimation::Ref<const beluga_estimation::SE2<T>> x,
      beluga_estimation::Ref<beluga_estimation::Vector3<T>> residual,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> jacobian = std::nullopt,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> hessian = std::nullopt) const {
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");

    // Compute the residual in the tangent space (3x1): log(x^-1 * z_obs).
    residual = -measurement_.CalculateResidual(x);

    // Compute Jacobian w.r.t x (3x3).
    if (jacobian) {
      // Jacobian of the measurement model H = -d/dx(log(z_obs^-1 * x)).
      // With r = log(x^-1 * z_obs) = -log(z_obs^-1 * x), this equals the
      // inverse right-Jacobian Jr^{-1}(-r) evaluated at r.
      // Ref: \cite sola2018micro, Eq. (70) (BCH), Eq. (79) (Log map), and App. C (SE2 specifics).
      auto& J = *jacobian;
      J.setIdentity();
      const T half_theta = residual(2) / static_cast<T>(2.0);
      J(0, 1) = half_theta;
      J(1, 0) = -half_theta;
      J(0, 2) = -residual(1) / static_cast<T>(2.0);
      J(1, 2) = residual(0) / static_cast<T>(2.0);
    }
  }

  /**
   * @brief Calculates the residual and its Jacobian in the tangent space (span interface).
   * r = log( x^-1 * z_obs )
   */
  void CalcResidual(
      view<std_future::span<const T>> dependent_variables,
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> residual,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_variables.size() == 0);
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 1);
    BELUGA_ESTIMATION_ASSERT(conditioning_variables[0].size() == 4);  // SE(2) ambient size.

    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> jacobian_opt;
    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> hessian_opt;

    if (jacobian) {
      jacobian_opt = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(*jacobian);
    }
    if (hessian) {
      hessian_opt = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(*hessian);
    }

    CalcResidual(
        beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_variables[0].data()),
        beluga_estimation::Map<beluga_estimation::Vector3<T>>(residual.data()), jacobian_opt, hessian_opt);
  }

 private:
  Identifier conditioning_id_;
  SE2NormalDistribution_<T> measurement_;
};

using SE2NormalPoseLikelihoodConstraint = SE2NormalPoseLikelihoodConstraint_<DefaultScalar>;

}  // namespace beluga_estimation

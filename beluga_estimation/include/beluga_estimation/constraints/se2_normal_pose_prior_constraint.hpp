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
 * @brief SE2 constraint representing an absolute prior pose with Gaussian noise.
 *
 * Models the prior x ~ N(prior_mean, prior_covariance).
 */
template <typename T>
class SE2NormalPosePriorConstraint_ : public LocallyNormalConstraint_<T> {
 public:
  SE2NormalPosePriorConstraint_(Identifier id, Identifier target_id, const SE2NormalDistribution_<T>& prior)
      : LocallyNormalConstraint_<T>(std::move(id)), target_id_{std::move(target_id)}, prior_{prior} {}

  bool HasDependentVariables() const override { return true; }
  bool HasConditioningVariables() const override { return false; }

  std_future::span<const Identifier> DependentVariables() const override { return {&target_id_, 1}; }

  std_future::span<const Identifier> ConditioningVariables() const override { return {}; }

  [[nodiscard]] std::size_t ResidualSize() const override { return beluga_estimation::SE2<T>::DoF; }

  T ComputeLogLikelihood(
      view<std_future::span<const T>> dependent_values,
      view<std_future::span<const T>> conditioning_values) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_values.size() == 1);
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 0);

    // Evaluate directly against the prior distribution
    return prior_.LogProbability(dependent_values[0]);
  }

  void TakeSamples(
      view<std_future::span<const T>> conditioning_values,
      view<view<std_future::span<T>>> sampled_dependent_values,
      RandomNumberGenerator& generator) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 0);
    BELUGA_ESTIMATION_ASSERT(sampled_dependent_values.size() == 1);

    // Sample directly into the output span from the prior
    prior_.Sample(sampled_dependent_values[0], generator);
  }

  /**
   * @brief Calculates the expected mean of the dependent variable (typed interface).
   * x_expected = prior.mean()
   *
   * @param mean Output: The expected mean pose.
   * @param jacobian Optional: Jacobian (not supported, throws if provided).
   * @param hessian Optional: Hessian (not supported, throws if provided).
   */
  void CalcMean(
      beluga_estimation::Ref<beluga_estimation::SE2<T>> mean,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> jacobian = std::nullopt,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> hessian = std::nullopt) const {
    BELUGA_ESTIMATION_DEMAND(
        !(jacobian || hessian), "jacobian and hessian computation are not supported for a prior constraint");

    mean = prior_.Mean();
  }

  /**
   * @brief Calculates the expected mean of the dependent variable (span interface).
   * x_expected = prior.mean()
   */
  void CalcMean(
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> mean,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.empty());

    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> jacobian_opt;
    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> hessian_opt;

    if (jacobian) {
      jacobian_opt = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(*jacobian);
    }
    if (hessian) {
      hessian_opt = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(*hessian);
    }

    auto mean_map = beluga_estimation::Map<beluga_estimation::SE2<T>>(mean.data());
    CalcMean(mean_map, jacobian_opt, hessian_opt);
  }

  /**
   * @brief Provides the tangent-space covariance of the prior (typed interface).
   *
   * @param covariance Output: The covariance matrix.
   */
  void CalcCovariance(beluga_estimation::Ref<beluga_estimation::Matrix3<T>> covariance) const {
    covariance = prior_.Covariance();
  }

  /**
   * @brief Provides the tangent-space covariance of the prior (span interface).
   */
  void CalcCovariance(
      [[maybe_unused]] view<std_future::span<const T>> conditioning_variables,
      std_future::mdspan<T, 2> covariance) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.empty());
    auto cov_map = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(covariance);
    CalcCovariance(cov_map);
  }

  /**
   * @brief Calculates the residual and its Jacobian in the tangent space (typed interface).
   * r = log( Prior^-1 * x )
   *
   * @param xj The target pose (dependent variable).
   * @param residual Output: The residual in tangent space (3x1).
   * @param jacobian Optional: Jacobian of residual w.r.t. xj (3x3).
   * @param hessian Optional: Hessian (not supported, throws if provided).
   */
  void CalcResidual(
      beluga_estimation::Ref<const beluga_estimation::SE2<T>> xj,
      beluga_estimation::Ref<beluga_estimation::Vector3<T>> residual,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> jacobian = std::nullopt,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> hessian = std::nullopt) const {
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");

    // Compute the residual in the tangent space (3x1)
    residual = prior_.CalculateResidual(xj);

    // Compute Jacobian w.r.t xj (3x3)
    if (jacobian) {
      // Jacobian of the residual r = Log(expected_xj^-1 * xj) w.r.t. xj.
      // This implements the first-order BCH approximation of the inverse right-Jacobian Jr^-1(r).
      // Ref: \cite sola2018micro, Eq. (70) (BCH), Eq. (79) (Log map), and App. C (SE2 specifics).
      auto& J = *jacobian;
      J.setIdentity();
      const T half_theta = residual(2) / static_cast<T>(2.0);
      J(0, 1) = -half_theta;
      J(1, 0) = half_theta;
      J(0, 2) = residual(1) / static_cast<T>(2.0);
      J(1, 2) = -residual(0) / static_cast<T>(2.0);
    }
  }

  /**
   * @brief Calculates the residual and its Jacobian in the tangent space (span interface).
   * r = log( Prior^-1 * x )
   */
  void CalcResidual(
      view<std_future::span<const T>> dependent_variables,
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> residual,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_variables.size() == 1);
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.empty());

    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> jacobian_opt;
    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> hessian_opt;

    if (jacobian) {
      jacobian_opt = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(*jacobian);
    }
    if (hessian) {
      hessian_opt = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(*hessian);
    }

    CalcResidual(
        beluga_estimation::Map<const beluga_estimation::SE2<T>>(dependent_variables[0].data()),
        beluga_estimation::Map<beluga_estimation::Vector3<T>>(residual.data()), jacobian_opt, hessian_opt);
  }

 private:
  Identifier target_id_;
  SE2NormalDistribution_<T> prior_;
};

using SE2NormalPosePriorConstraint = SE2NormalPosePriorConstraint_<DefaultScalar>;

}  // namespace beluga_estimation

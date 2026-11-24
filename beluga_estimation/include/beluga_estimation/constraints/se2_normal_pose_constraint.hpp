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
 * @brief SE2 constraint representing a relative transformation with Gaussian noise.
 *
 * Models the relationship x_j = x_i * z_ij * exp(noise),
 * where noise ~ N(0, covariance).
 */
template <typename T>
class SE2NormalPoseConstraint_ : public LocallyNormalConstraint_<T> {
 public:
  SE2NormalPoseConstraint_(
      Identifier id,
      Identifier source_id,
      Identifier target_id,
      const SE2NormalDistribution_<T>& measurement)
      : LocallyNormalConstraint_<T>(std::move(id)),
        source_id_{std::move(source_id)},
        target_id_{std::move(target_id)},
        measurement_{measurement} {}

  bool HasDependentVariables() const override { return true; }
  bool HasConditioningVariables() const override { return true; }

  std_future::span<const Identifier> DependentVariables() const override { return {&target_id_, 1}; }

  std_future::span<const Identifier> ConditioningVariables() const override { return {&source_id_, 1}; }

  [[nodiscard]] std::size_t ResidualSize() const override { return beluga_estimation::SE2<T>::DoF; }

  T ComputeLogLikelihood(
      view<std_future::span<const T>> dependent_values,
      view<std_future::span<const T>> conditioning_values) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_values.size() == 1);
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 1);

    const auto xi = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_values[0].data());

    // The mean of the dependent variable x_j is (x_i * measurement)
    const auto mean = xi * measurement_.Mean();
    SE2NormalDistribution_<T> dist(mean, measurement_.Covariance());

    // We pass the raw data of x_j (size 4) to the distribution
    return dist.LogProbability(dependent_values[0]);
  }

  void TakeSamples(
      view<std_future::span<const T>> conditioning_values,
      view<view<std_future::span<T>>> sampled_dependent_values,
      RandomNumberGenerator& generator) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 1);
    BELUGA_ESTIMATION_ASSERT(sampled_dependent_values.size() == 1);

    const auto xi = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_values[0].data());

    // The mean of the dependent variable x_j is (x_i * measurement)
    const auto mean = xi * measurement_.Mean();
    SE2NormalDistribution_<T> dist(mean, measurement_.Covariance());

    // Sample directly into the output span
    dist.Sample(sampled_dependent_values[0], generator);
  }

  /**
   * @brief Calculates the mean prediction (typed interface).
   * mean = xi * measurement
   *
   * @param xi The source pose (conditioning variable).
   * @param mean Output: The predicted mean pose.
   * @param jacobian Optional: Jacobian of mean w.r.t. xi (3x3).
   * @param hessian Optional: Hessian (not supported, throws if provided).
   */
  void CalcMean(
      beluga_estimation::Ref<const beluga_estimation::SE2<T>> xi,
      beluga_estimation::Ref<beluga_estimation::SE2<T>> mean,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> jacobian = std::nullopt,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> hessian = std::nullopt) const {
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");

    mean = xi * measurement_.Mean();

    if (jacobian) {
      // State transition Jacobian J = Ad(measurement^-1) for Kalman prediction.
      // Based on Eq. (65) and (99), using the identity in Eq. (33) from \cite sola2018micro.
      auto& J = *jacobian;
      J = measurement_.Mean().inverse().Adj();
    }
  }

  /**
   * @brief Calculates the expected mean of the dependent variable (span interface).
   * x_j_expected = x_i * measurement
   */
  void CalcMean(
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> mean,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 1);

    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> jacobian_opt;
    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> hessian_opt;

    if (jacobian) {
      jacobian_opt = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(*jacobian);
    }
    if (hessian) {
      hessian_opt = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(*hessian);
    }

    auto xi_map = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_variables[0].data());
    auto mean_map = beluga_estimation::Map<beluga_estimation::SE2<T>>(mean.data());
    CalcMean(xi_map, mean_map, jacobian_opt, hessian_opt);
  }

  /**
   * @brief Provides the tangent-space covariance (typed interface).
   *
   * @param covariance Output: The covariance matrix.
   */
  void CalcCovariance(beluga_estimation::Ref<beluga_estimation::Matrix3<T>> covariance) const {
    covariance = measurement_.Covariance();
  }

  /**
   * @brief Provides the tangent-space covariance (span interface).
   */
  void CalcCovariance(
      [[maybe_unused]] view<std_future::span<const T>> conditioning_variables,
      std_future::mdspan<T, 2> covariance) const override {
    auto cov_map = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(covariance);
    CalcCovariance(cov_map);
  }

  /**
   * @brief Calculates the residual and its Jacobian in the tangent space (typed interface).
   * r = log( (xi * measurement)^-1 * xj )
   *
   * @param xj The target pose (dependent variable).
   * @param xi The source pose (conditioning variable).
   * @param residual Output: The residual in tangent space (3x1).
   * @param jacobian Optional: Jacobian of residual w.r.t. xj (3x3).
   * @param hessian Optional: Hessian (not supported, throws if provided).
   */
  void CalcResidual(
      beluga_estimation::Ref<const beluga_estimation::SE2<T>> xj,
      beluga_estimation::Ref<const beluga_estimation::SE2<T>> xi,
      beluga_estimation::Ref<beluga_estimation::Vector3<T>> residual,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> jacobian = std::nullopt,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix3<T>>> hessian = std::nullopt) const {
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");

    // Compute the residual in the tangent space (3x1)
    residual = measurement_.CalculateResidual(xi.inverse() * xj);

    // Compute Jacobian if requested
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
   * r = log( (xi * measurement)^-1 * xj )
   */
  void CalcResidual(
      view<std_future::span<const T>> dependent_variables,
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> residual,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_variables.size() == 1);
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 1);

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
        beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_variables[0].data()),
        beluga_estimation::Map<beluga_estimation::Vector3<T>>(residual.data()), jacobian_opt, hessian_opt);
  }

 private:
  Identifier source_id_;
  Identifier target_id_;
  SE2NormalDistribution_<T> measurement_;
};

using SE2NormalPoseConstraint = SE2NormalPoseConstraint_<DefaultScalar>;

}  // namespace beluga_estimation

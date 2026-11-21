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

#include <array>
#include <cassert>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/locally_normal_constraint.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief Gaussian prior constraint over Euclidean space.
 *
 * Models a prior constraint p(x): x ~ N(μ, Σ)
 *
 * This constraint has only dependent variables (no conditioning variables).
 * It represents prior knowledge about a variable's distribution.
 *
 * @tparam T The underlying data type, typically float or double.
 * @tparam N The dimension of the variable. Can be Eigen::Dynamic for runtime dimension.
 */
template <typename T, int N = Eigen::Dynamic>
class NormalPriorConstraint_ : public LocallyNormalConstraint_<T> {
 public:
  using VectorType = beluga_estimation::Vector<T, N>;
  using MatrixType = beluga_estimation::Matrix<T, N, N>;

  /**
   * @brief Construct a prior constraint p(x).
   *
   * Models: x ~ N(prior_mean, prior_covariance)
   *
   * @param id The unique identifier for this constraint.
   * @param target_id The identifier of the dependent (target) variable x.
   * @param distribution The prior distribution.
   */
  NormalPriorConstraint_(
      Identifier id,
      Identifier target_id,
      std::shared_ptr<const NormalDistribution_<T, N>> distribution)
      : LocallyNormalConstraint_<T>(std::move(id)),
        target_id_{std::move(target_id)},
        distribution_{std::move(distribution)} {}

  bool HasDependentVariables() const override { return true; }

  bool HasConditioningVariables() const override { return false; }

  std_future::span<const Identifier> DependentVariables() const override {
    return std_future::span<const Identifier>{&target_id_, 1};
  }

  std_future::span<const Identifier> ConditioningVariables() const override {
    return std_future::span<const Identifier>{};
  }

  [[nodiscard]] std::size_t ResidualSize() const override {
    return static_cast<std::size_t>(distribution_->Mean().size());
  }

  /**
   * @brief Compute log-likelihood p(x) (Eigen::Ref version).
   *
   * @param x The dependent variable value.
   * @return The log-probability density.
   */
  T ComputeLogLikelihood(beluga_estimation::Ref<const VectorType> x) const { return distribution_->LogProbability(x); }

  /**
   * @brief Compute log-likelihood p(x) (span version).
   *
   * @param dependent_values Values of dependent variables (must contain exactly one span).
   * @param conditioning_values Values of conditioning variables (must be empty).
   * @return The log-probability density.
   */
  T ComputeLogLikelihood(
      view<std_future::span<const T>> dependent_values,
      view<std_future::span<const T>> conditioning_values) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_values.size() == 1);
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 0);
    return ComputeLogLikelihood(beluga_estimation::Map<const VectorType>(dependent_values[0]));
  }

  /**
   * @brief Sample from the prior distribution x ~ N(μ, Σ) (typed interface).
   *
   * Primary typed interface for generating multiple samples. Most efficient
   * for algorithms that work directly with Eigen types.
   *
   * @param x_samples A view of VectorType to store samples.
   * @param generator The random number generator.
   */
  void TakeSamples(beluga_estimation::view<VectorType&> x_samples, RandomNumberGenerator& generator) const {
    distribution_->Sample(x_samples, generator);
  }

  /**
   * @brief Sample from the prior distribution x ~ N(μ, Σ) (span interface).
   *
   * Accepts view of spans and delegates to distribution's span interface.
   *
   * @param x_samples A view of spans to store samples.
   * @param generator The random number generator.
   */
  void TakeSamples(beluga_estimation::view<std_future::span<T>> x_samples, RandomNumberGenerator& generator) const {
    distribution_->Sample(x_samples, generator);
  }

  /**
   * @brief Sample from the prior distribution x ~ N(μ, Σ) (span version).
   *
   * Generic interface for M samples from the prior. Each sample produces one dependent variable.
   *
   * @param conditioning_values Values of conditioning variables (must be empty for priors).
   * @param sampled_dependent_values M samples, each containing 1 dependent variable span.
   * @param generator The random number generator.
   */
  void TakeSamples(
      view<std_future::span<const T>> conditioning_values,
      view<view<std_future::span<T>>> sampled_dependent_values,
      RandomNumberGenerator& generator) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 0);
    BELUGA_ESTIMATION_ASSERT(sampled_dependent_values.size() == 1);
    // Extract the single dependent variable span from each sample
    const auto expected_size = distribution_->Mean().size();
    auto samples = ranges::views::transform(sampled_dependent_values[0], [&](auto sample) {
      BELUGA_ESTIMATION_ASSERT(sample.size() == expected_size);
      return beluga_estimation::Map<VectorType>(sample);
    });
    distribution_->Sample(samples, generator);
  }

  /**
   * @brief Calculate the expected mean (returns prior mean) (Eigen::Ref version).
   *
   * For a prior constraint, the mean is simply the prior mean μ.
   * No Jacobian is computed since the mean doesn't depend on any conditioning variables.
   *
   * @param mean Output vector for the mean.
   */
  void CalcMean(beluga_estimation::Ref<VectorType> mean) const { mean = distribution_->Mean(); }

  /**
   * @brief Calculate the expected mean (returns prior mean) (span version).
   *
   * @param conditioning_variables Values of conditioning variables (must be empty).
   * @param mean_span Output span for the mean.
   * @param jacobian_span Optional Jacobian (throws if requested).
   * @param hessian_span Optional Hessian (throws if requested).
   *
   * @throws std::runtime_error if jacobian or hessian is requested.
   */
  void CalcMean(
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> mean_span,
      std::optional<std_future::mdspan<T, 2>> jacobian_span = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian_span = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 0);

    BELUGA_ESTIMATION_DEMAND(!jacobian_span, "jacobian computation is not supported for a prior constraint");
    BELUGA_ESTIMATION_DEMAND(!hessian_span, "hessian computation is not supported");

    beluga_estimation::Map<VectorType> mean_map(mean_span);
    CalcMean(mean_map);
  }

  /**
   * @brief Provide the prior covariance matrix (Eigen::Ref version).
   *
   * @param covariance Output matrix for the covariance.
   */
  void CalcCovariance(beluga_estimation::Ref<MatrixType> covariance) const { covariance = distribution_->Covariance(); }

  /**
   * @brief Provide the prior covariance matrix (span version).
   *
   * @param conditioning_variables Values of conditioning variables (unused).
   * @param covariance_span Output mdspan for the covariance matrix.
   */
  void CalcCovariance(
      [[maybe_unused]] view<std_future::span<const T>> conditioning_variables,
      std_future::mdspan<T, 2> covariance_span) const override {
    auto covariance_map = beluga_estimation::Map<MatrixType>(covariance_span);
    CalcCovariance(covariance_map);
  }

  /**
   * @brief Calculate the residual: r = x - μ (Eigen::Ref version).
   *
   * @param x The actual dependent variable value.
   * @param residual Output vector for the residual.
   * @param jacobian Optional Jacobian matrix (identity if provided).
   */
  void CalcResidual(
      beluga_estimation::Ref<const VectorType> x,
      beluga_estimation::Ref<VectorType> residual,
      beluga_estimation::Ref<MatrixType> jacobian) const {
    residual = x - distribution_->Mean();
    jacobian.setIdentity();
  }

  /**
   * @brief Calculate the residual: r = x - μ (Eigen::Ref version without Jacobian).
   *
   * @param x The actual dependent variable value.
   * @param residual Output vector for the residual.
   */
  void CalcResidual(beluga_estimation::Ref<const VectorType> x, beluga_estimation::Ref<VectorType> residual) const {
    residual = x - distribution_->Mean();
  }

  /**
   * @brief Calculate the residual: r = x - μ (span version).
   *
   * @param dependent_variables Actual values of dependent variables (must contain exactly one span).
   * @param conditioning_variables Values of conditioning variables (must be empty).
   * @param residual_span Output span for the residual.
   * @param jacobian_span Optional Jacobian matrix (identity if provided).
   * @param hessian_span Optional Hessian (throws if requested).
   *
   * @throws std::runtime_error if hessian is requested.
   */
  void CalcResidual(
      view<std_future::span<const T>> dependent_variables,
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> residual_span,
      std::optional<std_future::mdspan<T, 2>> jacobian_span = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian_span = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_variables.size() == 1);
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 0);

    BELUGA_ESTIMATION_DEMAND(!hessian_span, "hessian computation is not supported");

    const int dim = distribution_->Mean().size();
    const auto x = beluga_estimation::Map<const VectorType>(dependent_variables[0].data(), dim);
    auto residual_map = beluga_estimation::Map<VectorType>(residual_span.data(), dim);

    if (jacobian_span) {
      auto jacobian_map = beluga_estimation::Map<MatrixType>(*jacobian_span);
      CalcResidual(x, residual_map, jacobian_map);
    } else {
      CalcResidual(x, residual_map);
    }
  }

 private:
  Identifier target_id_;
  std::shared_ptr<const NormalDistribution_<T, N>> distribution_;
};

// Type aliases
template <int N = Eigen::Dynamic>
using NormalPriorConstraint = NormalPriorConstraint_<DefaultScalar, N>;

using NormalPriorConstraint2 = NormalPriorConstraint_<DefaultScalar, 2>;
using NormalPriorConstraint3 = NormalPriorConstraint_<DefaultScalar, 3>;

}  // namespace beluga_estimation

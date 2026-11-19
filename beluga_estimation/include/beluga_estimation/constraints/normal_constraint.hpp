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

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/locally_normal_constraint.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief Gaussian binary constraint over Euclidean space.
 *
 * Models a binary constraint p(x|y): x ~ y + z where z ~ N(μ, Σ)
 *
 * This constraint has both dependent and conditioning variables.
 * It represents a relationship between two variables with additive Gaussian noise.
 *
 * @tparam T The underlying data type, typically float or double.
 * @tparam N The dimension of the variables. Can be Eigen::Dynamic for runtime dimension.
 */
template <typename T, int N = Eigen::Dynamic>
class NormalConstraint_ : public LocallyNormalConstraint_<T> {
 public:
  using VectorType = beluga_estimation::Vector<T, N>;
  using MatrixType = beluga_estimation::Matrix<T, N, N>;

  /**
   * @brief Construct a binary constraint p(x|y).
   *
   * Models: x = y + z where z ~ N(measurement_mean, measurement_covariance)
   *
   * @param id The unique identifier for this constraint.
   * @param source_id The identifier of the conditioning (source) variable y.
   * @param target_id The identifier of the dependent (target) variable x.
   * @param distribution The measurement distribution.
   */
  NormalConstraint_(
      Identifier id,
      Identifier source_id,
      Identifier target_id,
      std::shared_ptr<const NormalDistribution_<T, N>> distribution)
      : LocallyNormalConstraint_<T>(std::move(id)),
        source_id_{std::move(source_id)},
        target_id_{std::move(target_id)},
        distribution_{std::move(distribution)} {}

  bool HasDependentVariables() const override { return true; }

  bool HasConditioningVariables() const override { return true; }

  std_future::span<const Identifier> DependentVariables() const override {
    return std_future::span<const Identifier>{&target_id_, 1};
  }

  std_future::span<const Identifier> ConditioningVariables() const override {
    return std_future::span<const Identifier>{&source_id_, 1};
  }

  [[nodiscard]] std::size_t ResidualSize() const override {
    return static_cast<std::size_t>(distribution_->Mean().size());
  }

  /**
   * @brief Compute log-likelihood p(x|y) (Eigen::Ref version).
   *
   * Evaluates the conditional probability where x ~ N(y + μ, Σ).
   *
   * @param x The dependent variable value.
   * @param y The conditioning variable value.
   * @return The log-probability density.
   */
  T ComputeLogLikelihood(beluga_estimation::Ref<const VectorType> x, beluga_estimation::Ref<const VectorType> y) const {
    return distribution_->LogProbability(x - y);
  }

  /**
   * @brief Compute log-likelihood p(x|y) (span version).
   *
   * @param dependent_values Values of dependent variables (must contain exactly one span).
   * @param conditioning_values Values of conditioning variables (must contain exactly one span).
   * @return The log-probability density.
   */
  T ComputeLogLikelihood(
      view<std_future::span<const T>> dependent_values,
      view<std_future::span<const T>> conditioning_values) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_values.size() == 1);
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 1);
    BELUGA_ESTIMATION_DEMAND(
        dependent_values[0].size() == distribution_->Mean().size(),
        "dependent variable dimension does not match the distribution");
    BELUGA_ESTIMATION_DEMAND(
        conditioning_values[0].size() == distribution_->Mean().size(),
        "conditioning variable dimension does not match the distribution");
    const auto x = beluga_estimation::Map<const VectorType>(dependent_values[0]);
    const auto y = beluga_estimation::Map<const VectorType>(conditioning_values[0]);
    return ComputeLogLikelihood(x, y);
  }

  /**
   * @brief Sample x = y + z where z ~ N(μ, Σ) (Eigen::Ref version).
   *
   * @tparam Generator A Uniform Random Bit Generator type.
   * @param y The conditioning variable value.
   * @param x_sample Output vector to store the sampled value.
   * @param generator The random number generator.
   */
  void TakeSamples(
      beluga_estimation::Ref<const VectorType> y,
      view<VectorType&> samples,
      RandomNumberGenerator& generator) const {
    distribution_->Sample(samples, generator);
    for (auto& x : samples) {
      x = y + x;
    }
  }

  /**
   * @brief Sample x = y + z where z ~ N(μ, Σ) (span version).
   *
   * @param conditioning_values Values of conditioning variables (must contain exactly one span).
   * @param sampled_dependent_values Output spans for sampled values (must contain exactly one span).
   * @param generator The random number generator.
   */
  void TakeSamples(
      view<std_future::span<const T>> conditioning_values,
      view<view<std_future::span<T>>> sampled_dependent_values,
      RandomNumberGenerator& generator) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 1);
    BELUGA_ESTIMATION_DEMAND(
        conditioning_values[0].size() == distribution_->Mean().size(),
        "conditioning variable dimension does not match the distribution");
    BELUGA_ESTIMATION_ASSERT(sampled_dependent_values.size() == 1);
    const auto y = beluga_estimation::Map<const VectorType>(conditioning_values[0]);
    const auto expected_size = distribution_->Mean().size();
    beluga_estimation::view<beluga_estimation::Map<VectorType>> samples =
        ranges::views::transform(sampled_dependent_values[0], [&](auto sample) {
          BELUGA_ESTIMATION_DEMAND(
              sample.size() == expected_size, "sampled value dimension does not match the distribution");
          return beluga_estimation::Map<VectorType>(sample);
        });
    distribution_->Sample(samples, generator);
    for (auto x : samples) {
      x = y + x;
    }
  }

  /**
   * @brief Calculate the expected mean x_expected = y + μ (Eigen::Ref version).
   *
   * @param y The conditioning variable value.
   * @param mean Output vector for the expected mean.
   * @param jacobian Optional Jacobian matrix (identity: dx/dy = I).
   */
  void CalcMean(
      beluga_estimation::Ref<const VectorType> y,
      beluga_estimation::Ref<VectorType> mean,
      beluga_estimation::Ref<MatrixType> jacobian) const {
    CalcMean(y, mean);
    jacobian.setIdentity();
  }

  /**
   * @brief Calculate the expected mean x_expected = y + μ (Eigen::Ref version without Jacobian).
   *
   * @param y The conditioning variable value.
   * @param mean Output vector for the expected mean.
   */
  void CalcMean(beluga_estimation::Ref<const VectorType> y, beluga_estimation::Ref<VectorType> mean) const {
    mean = y + distribution_->Mean();
  }

  /**
   * @brief Calculate the expected mean x_expected = y + μ (span version).
   *
   * @param conditioning_variables Values of conditioning variables (must contain exactly one span).
   * @param mean_span Output span for the mean.
   * @param jacobian_span Optional Jacobian matrix (identity if provided).
   * @param hessian_span Optional Hessian (throws if requested).
   *
   * @throws std::runtime_error if hessian is requested.
   */
  void CalcMean(
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> mean_span,
      std::optional<std_future::mdspan<T, 2>> jacobian_span = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian_span = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 1);

    BELUGA_ESTIMATION_DEMAND(!hessian_span, "hessian computation is not supported");

    BELUGA_ESTIMATION_DEMAND(
        conditioning_variables[0].size() == distribution_->Mean().size(),
        "conditioning variable dimension does not match the distribution");
    BELUGA_ESTIMATION_ASSERT(mean_span.size() == distribution_->Mean().size());
    const auto y = beluga_estimation::Map<const VectorType>(conditioning_variables[0]);
    auto mean_map = beluga_estimation::Map<VectorType>(mean_span);

    if (jacobian_span) {
      BELUGA_ESTIMATION_ASSERT(jacobian_span->extent(0) == distribution_->Mean().size());
      BELUGA_ESTIMATION_ASSERT(jacobian_span->extent(1) == distribution_->Mean().size());
      auto jacobian_map = beluga_estimation::Map<MatrixType>(*jacobian_span);
      CalcMean(y, mean_map, jacobian_map);
    } else {
      CalcMean(y, mean_map);
    }
  }

  /**
   * @brief Provide the measurement covariance matrix (Eigen::Ref version).
   *
   * @param covariance Output matrix for the covariance.
   */
  void CalcCovariance(beluga_estimation::Ref<MatrixType> covariance) const { covariance = distribution_->Covariance(); }

  /**
   * @brief Provide the measurement covariance matrix (span version).
   *
   * @param conditioning_variables Values of conditioning variables (unused).
   * @param covariance_span Output mdspan for the covariance matrix.
   */
  void CalcCovariance(
      [[maybe_unused]] view<std_future::span<const T>> conditioning_variables,
      std_future::mdspan<T, 2> covariance_span) const override {
    BELUGA_ESTIMATION_ASSERT(covariance_span.extent(0) == distribution_->Covariance().rows());
    BELUGA_ESTIMATION_ASSERT(covariance_span.extent(1) == distribution_->Covariance().cols());
    auto covariance_map = beluga_estimation::Map<MatrixType>(covariance_span);
    CalcCovariance(covariance_map);
  }

  /**
   * @brief Calculate the residual: r = x - (y + μ) (Eigen::Ref version).
   *
   * @param x The actual dependent variable value.
   * @param y The conditioning variable value.
   * @param residual Output vector for the residual.
   * @param jacobian Optional Jacobian matrix (identity: dr/dx = I).
   */
  void CalcResidual(
      beluga_estimation::Ref<const VectorType> x,
      beluga_estimation::Ref<const VectorType> y,
      beluga_estimation::Ref<VectorType> residual,
      beluga_estimation::Ref<MatrixType> jacobian) const {
    CalcResidual(x, y, residual);
    jacobian.setIdentity();
  }

  /**
   * @brief Calculate the residual: r = x - (y + μ) (Eigen::Ref version without Jacobian).
   *
   * @param x The actual dependent variable value.
   * @param y The conditioning variable value.
   * @param residual Output vector for the residual.
   */
  void CalcResidual(
      beluga_estimation::Ref<const VectorType> x,
      beluga_estimation::Ref<const VectorType> y,
      beluga_estimation::Ref<VectorType> residual) const {
    residual = x - (y + distribution_->Mean());
  }

  /**
   * @brief Calculate the residual: r = x - (y + μ) (span version).
   *
   * @param dependent_variables Actual values of dependent variables (must contain exactly one span).
   * @param conditioning_variables Values of conditioning variables (must contain exactly one span).
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
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 1);

    BELUGA_ESTIMATION_DEMAND(!hessian_span, "hessian computation is not supported");

    BELUGA_ESTIMATION_DEMAND(
        dependent_variables[0].size() == distribution_->Mean().size(),
        "dependent variable dimension does not match the distribution");
    BELUGA_ESTIMATION_DEMAND(
        conditioning_variables[0].size() == distribution_->Mean().size(),
        "conditioning variable dimension does not match the distribution");
    BELUGA_ESTIMATION_ASSERT(residual_span.size() == distribution_->Mean().size());
    const auto x = beluga_estimation::Map<const VectorType>(dependent_variables[0]);
    const auto y = beluga_estimation::Map<const VectorType>(conditioning_variables[0]);
    auto residual_map = beluga_estimation::Map<VectorType>(residual_span);

    if (jacobian_span) {
      BELUGA_ESTIMATION_ASSERT(jacobian_span->extent(0) == distribution_->Mean().size());
      BELUGA_ESTIMATION_ASSERT(jacobian_span->extent(1) == distribution_->Mean().size());
      auto jacobian_map = beluga_estimation::Map<MatrixType>(*jacobian_span);
      CalcResidual(x, y, residual_map, jacobian_map);
    } else {
      CalcResidual(x, y, residual_map);
    }
  }

 private:
  Identifier source_id_;
  Identifier target_id_;
  std::shared_ptr<const NormalDistribution_<T, N>> distribution_;
};

// Type aliases
template <int N = Eigen::Dynamic>
using NormalConstraint = NormalConstraint_<DefaultScalar, N>;

using NormalConstraint2 = NormalConstraint_<DefaultScalar, 2>;
using NormalConstraint3 = NormalConstraint_<DefaultScalar, 3>;

}  // namespace beluga_estimation

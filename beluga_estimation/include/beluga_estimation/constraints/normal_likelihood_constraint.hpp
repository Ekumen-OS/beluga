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

#include <cstddef>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <utility>

#include <Eigen/Core>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/locally_normal_constraint.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief Gaussian likelihood constraint over Euclidean space.
 *
 * Models a likelihood constraint p(.|y) over a Euclidean conditioning variable
 * y, with z_obs a fixed observation:
 *
 * @f[
 *   p(z_{obs}|y) = \mathcal{N}(z_{obs}; P y, \Sigma)
 * @f]
 *
 * where P is the projection matrix (the measurement model Jacobian
 * @f$\frac{\partial h}{\partial y}@f$, with @f$h(y) = P y@f$) and @f$\Sigma@f$
 * is the measurement covariance. P has one row per fused measurement component
 * and one column per conditioning-variable coordinate; it need not be a
 * selection, so linear combinations of coordinates are supported.
 *
 * Only fused measurement components are modelled, so the distribution is
 * expected to be already projected onto them: callers reduce a whole
 * measurement distribution once, before constructing the constraint (for
 * Gaussian measurements projecting a subset of components is an exact
 * marginalization). The constraint keeps that distribution whole and expresses
 * the relation to the variable as one linear map.
 *
 * This constraint has only conditioning variables (no dependent variables).
 * Without a projection matrix, the measurement observes the whole variable and
 * the constraint uses the identity map.
 *
 * @tparam T The underlying data type, typically float or double.
 * @tparam N The dimension of the measurement distribution. Can be Eigen::Dynamic for runtime dimension.
 */
template <typename T, int N = Eigen::Dynamic>
class NormalLikelihoodConstraint_ : public LocallyNormalConstraint_<T> {
 public:
  using VectorType = beluga_estimation::Vector<T, N>;
  using MatrixType = beluga_estimation::Matrix<T, N, N>;

  /**
   * @brief Construct a likelihood constraint p(.|y) observing the whole variable.
   *
   * Models: p(z_obs|y) where z_obs is the observed measurement stored in distribution.
   * The distribution mean is z_obs, covariance is measurement uncertainty.
   *
   * @param id The unique identifier for this constraint.
   * @param conditioning_id The identifier of the conditioning variable y.
   * @param distribution The measurement distribution (stores observed value and uncertainty).
   */
  NormalLikelihoodConstraint_(
      Identifier id,
      Identifier conditioning_id,
      std::shared_ptr<const NormalDistribution_<T, N>> distribution)
      : LocallyNormalConstraint_<T>(std::move(id)),
        conditioning_id_{std::move(conditioning_id)},
        distribution_{std::move(distribution)},
        projection_{MatrixX<T>::Identity(distribution_->Mean().size(), distribution_->Mean().size())} {}

  /**
   * @brief Construct a likelihood constraint p(.|y) observing the variable through a projection.
   *
   * The distribution must describe only the fused measurement components;
   * project a whole measurement distribution onto them beforehand. The
   * projection matrix maps conditioning-variable coordinates onto that
   * measurement space, so that @f$p(z_{obs}|y) = \mathcal{N}(z_{obs}; P y,
   * \Sigma)@f$.
   *
   * Row i of the projection matrix applies to the i-th component of the stored
   * distribution; it is up to the caller to keep the two aligned.
   *
   * @param id The unique identifier for this constraint.
   * @param conditioning_id The identifier of the conditioning variable y.
   * @param distribution The distribution of the fused measurement components.
   * @param projection The projection matrix P, as many rows as the distribution has dimensions and as many
   * columns as the conditioning variable has coordinates.
   */
  NormalLikelihoodConstraint_(
      Identifier id,
      Identifier conditioning_id,
      std::shared_ptr<const NormalDistribution_<T, N>> distribution,
      MatrixX<T> projection)
      : LocallyNormalConstraint_<T>(std::move(id)),
        conditioning_id_{std::move(conditioning_id)},
        distribution_{std::move(distribution)},
        projection_{std::move(projection)} {
    BELUGA_ESTIMATION_DEMAND(projection_.rows() == distribution_->Mean().size());
  }

  bool HasDependentVariables() const override { return false; }

  bool HasConditioningVariables() const override { return true; }

  std_future::span<const Identifier> DependentVariables() const override {
    return std_future::span<const Identifier>{};
  }

  std_future::span<const Identifier> ConditioningVariables() const override {
    return std_future::span<const Identifier>{&conditioning_id_, 1};
  }

  [[nodiscard]] std::size_t ResidualSize() const override { return Count(); }

  /**
   * @brief Compute log-likelihood p(z_obs|y) (typed interface).
   *
   * Evaluates @f$\mathcal{N}(z_{obs}; P y, \Sigma)@f$.
   *
   * @param y The conditioning variable value.
   * @return The log-probability density.
   */
  T ComputeLogLikelihood(beluga_estimation::Ref<const beluga_estimation::VectorX<T>> y) const {
    BELUGA_ESTIMATION_DEMAND(projection_.cols() == y.size());
    return distribution_->LogProbability(projection_ * y);
  }

  /**
   * @brief Compute log-likelihood p(z_obs|y) (span version).
   *
   * @param dependent_values Values of dependent variables (must be empty).
   * @param conditioning_values Values of conditioning variables (must contain exactly one span).
   * @return The log-probability density.
   */
  T ComputeLogLikelihood(
      view<std_future::span<const T>> dependent_values,
      view<std_future::span<const T>> conditioning_values) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_values.size() == 0);
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 1);
    const auto& values = conditioning_values[0];
    const auto y = beluga_estimation::Map<const beluga_estimation::VectorX<T>>(
        values.data(), static_cast<Eigen::Index>(values.size()));
    return ComputeLogLikelihood(y);
  }

  /**
   * @brief Sampling is not supported for likelihood constraints.
   *
   * @throws std::runtime_error always.
   */
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
      [[maybe_unused]] view<std_future::span<const T>> conditioning_variables,
      [[maybe_unused]] std_future::span<T> mean_span,
      [[maybe_unused]] std::optional<std_future::mdspan<T, 2>> jacobian_span = std::nullopt,
      [[maybe_unused]] std::optional<std_future::mdspan<T, 2>> hessian_span = std::nullopt) const override {
    BELUGA_ESTIMATION_DEMAND(false, "mean calculation is not supported for likelihood constraints");
  }

  /**
   * @brief Provide the measurement covariance matrix (typed interface).
   *
   * Returns the covariance of the fused measurement components.
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
    auto covariance_map = beluga_estimation::Map<MatrixType>(covariance_span);
    CalcCovariance(covariance_map);
  }

  /**
   * @brief Calculate the residual: r = z_obs - P y (typed interface).
   *
   * The residual is the innovation `measurement - h(state)` with h(y) = P y.
   * The optional Jacobian is the measurement model Jacobian d h/d y = P (note
   * this is the negative of the residual's own derivative, consistent with the
   * EKF update `x += K * r`).
   *
   * @param y The conditioning variable value.
   * @param residual Output vector for the residual.
   * @param jacobian Optional Jacobian matrix (the projection matrix).
   */
  void CalcResidual(
      beluga_estimation::Ref<const beluga_estimation::VectorX<T>> y,
      beluga_estimation::Ref<VectorType> residual,
      beluga_estimation::Ref<beluga_estimation::MatrixX<T>> jacobian) const {
    CalcResidual(y, residual);
    jacobian = projection_;
  }

  /**
   * @brief Calculate the residual: r = z_obs - P y (typed interface without Jacobian).
   *
   * @param y The conditioning variable value.
   * @param residual Output vector for the residual.
   */
  void CalcResidual(
      beluga_estimation::Ref<const beluga_estimation::VectorX<T>> y,
      beluga_estimation::Ref<VectorType> residual) const {
    BELUGA_ESTIMATION_DEMAND(projection_.cols() == y.size());
    BELUGA_ESTIMATION_DEMAND(residual.size() == static_cast<Eigen::Index>(Count()));
    residual = distribution_->Mean() - projection_ * y;
  }

  /**
   * @brief Calculate the residual: r = z_obs - P y (span version).
   *
   * @param dependent_variables Actual values of dependent variables (must be empty).
   * @param conditioning_variables Values of conditioning variables (must contain exactly one span).
   * @param residual_span Output span for the residual.
   * @param jacobian_span Optional Jacobian matrix (the projection matrix).
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
    BELUGA_ESTIMATION_ASSERT(dependent_variables.size() == 0);
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 1);

    BELUGA_ESTIMATION_DEMAND(!hessian_span, "hessian computation is not supported");

    const auto& values = conditioning_variables[0];
    const auto y = beluga_estimation::Map<const beluga_estimation::VectorX<T>>(
        values.data(), static_cast<Eigen::Index>(values.size()));
    auto residual =
        beluga_estimation::Map<VectorType>(residual_span.data(), static_cast<Eigen::Index>(residual_span.size()));

    if (jacobian_span) {
      auto jacobian = beluga_estimation::Map<beluga_estimation::MatrixX<T>>(*jacobian_span);
      CalcResidual(y, residual, jacobian);
    } else {
      CalcResidual(y, residual);
    }
  }

 private:
  /// @brief Number of fused measurement components.
  std::size_t Count() const { return static_cast<std::size_t>(distribution_->Mean().size()); }

  Identifier conditioning_id_;
  std::shared_ptr<const NormalDistribution_<T, N>> distribution_;
  MatrixX<T> projection_;
};

// Type aliases
template <int N = Eigen::Dynamic>
using NormalLikelihoodConstraint = NormalLikelihoodConstraint_<DefaultScalar, N>;

using NormalLikelihoodConstraint2 = NormalLikelihoodConstraint_<DefaultScalar, 2>;
using NormalLikelihoodConstraint3 = NormalLikelihoodConstraint_<DefaultScalar, 3>;

}  // namespace beluga_estimation

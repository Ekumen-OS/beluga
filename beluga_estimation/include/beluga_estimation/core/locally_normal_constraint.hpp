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

#include <optional>

#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/constraint.hpp>

namespace beluga_estimation {

/**
 * @brief Interface for constraints with locally normal distributions.
 *
 * This interface extends the base Constraint interface with methods suitable
 * for Extended Kalman Filters (EKF) and similar algorithms that rely on
 * local linear-Gaussian approximations.
 *
 * Locally normal constraints provide:
 * - Mean calculation (expected value of dependent variables)
 * - Covariance calculation (uncertainty in tangent space)
 * - Residual calculation (error between expected and actual values)
 * - Jacobian computation (for linearization)
 *
 * @tparam T The underlying data type for the constraint, typically float or double.
 */
template <typename T>
class LocallyNormalConstraint_ : public Constraint_<T> {
 public:
  using Constraint_<T>::Constraint_;

  /**
   * @brief Calculate the expected mean of dependent variable(s).
   *
   * Computes the expected value of the dependent variables given the
   * conditioning variables (if any). For prior constraints, the
   * conditioning_variables view will be empty.
   *
   * @param conditioning_variables Values of conditioning variables (empty for priors).
   * @param mean Output span for the expected mean.
   * @param jacobian Optional Jacobian matrix (row-major layout) of the mean
   *                 w.r.t. conditioning variables. Used in Kalman prediction.
   * @param hessian Optional Hessian (typically not supported, throws if provided).
   *
   * @throws std::runtime_error if hessian is requested but not supported.
   */
  virtual void CalcMean(
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> mean,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const = 0;

  /**
   * @brief Calculate the covariance of dependent variable(s) in tangent space.
   *
   * Returns the covariance matrix representing the uncertainty of the
   * dependent variables. The covariance is expressed in the tangent space
   * for manifold-valued variables.
   *
   * @param conditioning_variables Values of conditioning variables (may be empty).
   * @param covariance Output mdspan for covariance matrix (row-major layout).
   */
  virtual void CalcCovariance(
      view<std_future::span<const T>> conditioning_variables,
      std_future::mdspan<T, 2> covariance) const = 0;

  /**
   * @brief Calculate the residual between actual and expected values.
   *
   * Computes the residual (error) in tangent space:
   * residual = actual - expected
   *
   * For manifold-valued variables, the residual is computed using the
   * manifold's logarithmic map. For Euclidean spaces, it's simple subtraction.
   *
   * @param dependent_variables Actual values of dependent variables.
   * @param conditioning_variables Values of conditioning variables (empty for priors).
   * @param residual Output span for residual vector.
   * @param jacobian Optional Jacobian matrix (row-major layout) of the residual
   *                 w.r.t. dependent variables. Used in Kalman update.
   * @param hessian Optional Hessian (typically not supported, throws if provided).
   *
   * @throws std::runtime_error if hessian is requested but not supported.
   */
  virtual void CalcResidual(
      view<std_future::span<const T>> dependent_variables,
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> residual,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const = 0;

  /**
   * @brief Returns the dimension of the residual vector in tangent space.
   * @return The number of components in the residual vector.
   */
  [[nodiscard]] virtual std::size_t ResidualSize() const = 0;
};

using LocallyNormalConstraint = LocallyNormalConstraint_<DefaultScalar>;

}  // namespace beluga_estimation

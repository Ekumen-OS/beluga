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

#include <random>
#include <vector>

#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/identifiable.hpp>

namespace beluga_estimation {

/**
 * @brief Interface for constraints in the DBN.
 * Constraints define relationships between variables, such as measurement models
 * or process models.
 */
template <typename T>
class Constraint_ : public Identifiable {
 public:
  using Identifiable::Identifiable;

  /**
   * @brief Check if the constraint has dependent and conditioning variables.
   */
  virtual bool HasDependentVariables() const = 0;

  /**
   * @brief Check if the constraint has conditioning variables.
   */
  virtual bool HasConditioningVariables() const = 0;

  /**
   * @brief Access the dependent variables associated with this constraint.
   */
  virtual std_future::span<const Identifier> DependentVariables() const = 0;

  /**
   * @brief Access the conditioning variables associated with this constraint.
   */
  virtual std_future::span<const Identifier> ConditioningVariables() const = 0;

  /**
   * @brief Compute the log-likelihood of dependent values given conditioning values.
   *
   * @param dependent_values Values of dependent variables.
   * @param conditioning_values Values of conditioning variables.
   * @return Log-likelihood value.
   */
  virtual T ComputeLogLikelihood(
      view<std_future::span<const T>> dependent_values,
      view<std_future::span<const T>> conditioning_values) const = 0;

  /**
   * @brief Generate samples of dependent variables given conditioning values.
   *
   * @param conditioning_values Values of conditioning variables.
   * @param sampled_dependent_values Samples of dependent variables,
   * following a structure-of-arrays pattern for performance. That is,
   * each element in the outer view corresponds to a view over samples
   * for one dependent variable.
   * @param generator The random number generator used for sampling.
   */
  virtual void TakeSamples(
      view<std_future::span<const T>> conditioning_values,
      view<view<std_future::span<T>>> sampled_dependent_values,
      RandomNumberGenerator& generator) const = 0;
};

using Constraint = Constraint_<DefaultScalar>;

}  // namespace beluga_estimation

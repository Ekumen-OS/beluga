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

#include <memory>

#include <beluga_estimation/core/distribution.hpp>
#include <beluga_estimation/core/identifiable.hpp>
#include <beluga_estimation/core/manifold.hpp>

namespace beluga_estimation {

template <typename T>
class Estimate_;

/**
 * @brief Interface for variables in the DBN.
 * Each variable is associated with a manifold and a distribution, enabling
 * inference and optimization over complex state spaces.
 */
template <typename T>
class Variable_ : public Identifiable {
 public:
  using Identifiable::Identifiable;
  /**
   * @brief Access the manifold associated with this variable.
   * @return A shared pointer to the manifold (singleton or owned).
   */
  virtual std::shared_ptr<const beluga_estimation::Manifold_<T>> Manifold() const = 0;

  /**
   * @brief Extracts this variable's distribution from an estimate.
   *
   * Marginalizes the estimate to this variable's identifier and constructs
   * a typed distribution from the result. For Euclidean variables, the
   * returned distribution is the inner @ref EuclideanDistribution_ directly.
   * For manifold variables (e.g. SE(2)), constructs a specialized wrapper
   * (e.g. @ref SE2TangentDistribution_).
   *
   * Returns nullptr if marginalization is not supported or if the
   * marginalized estimate does not contain a compatible distribution.
   *
   * @param estimate The estimate to extract from. Must contain this variable.
   * @return A shared pointer to the extracted distribution, or nullptr if
   *   the estimate cannot be marginalized.
   */
  std::shared_ptr<const beluga_estimation::Distribution_<T>> GetDistribution(const Estimate_<T>& estimate) const {
    auto marginal_estimate = estimate.Marginalize(this->Id());
    if (!marginal_estimate) {
      return nullptr;
    }
    return marginal_estimate->GetDistribution();
  }
};

using Variable = Variable_<DefaultScalar>;

}  // namespace beluga_estimation

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
#include <utility>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/estimate.hpp>
#include <beluga_estimation/core/on_manifold_estimate.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/core/variable.hpp>
#include <beluga_estimation/distributions/se2_tangent_distribution.hpp>
#include <beluga_estimation/manifolds/se2_manifold.hpp>

namespace beluga_estimation {

/**
 * @brief A variable representing a 2D pose on the $SE(2)$ manifold.
 *
 * This class binds together a unique identifier with the structural
 * properties of the $SE(2)$ manifold (via `SE2Manifold`). It does not
 * store a distribution directly — the distribution is extracted from
 * an @ref Estimate_ via @ref GetDistribution.
 *
 * @tparam T The scalar type of the variable (e.g., double, float).
 */
template <typename T>
class SE2Variable_ : public Variable_<T> {
 public:
  /**
   * @brief Constructs a new $SE(2)$ variable.
   * @param id The unique identifier for this variable node.
   */
  explicit SE2Variable_(Identifier id) : Variable_<T>(std::move(id)) {}

  /**
   * @brief Retrieves the manifold associated with this variable type.
   * @return A shared pointer to the global `SE2Manifold_<T>` singleton instance.
   */
  std::shared_ptr<const beluga_estimation::Manifold_<T>> Manifold() const override {
    return SE2Manifold_<T>::Instance();
  }

  /**
   * @brief Extracts the SE(2) distribution from an @ref OnManifoldEstimate_
   *        (typed getter).
   *
   * Extracts the SE(2) origin and tangent-space Euclidean distribution
   * and constructs an @ref SE2TangentDistribution_.
   *
   * @param on_manifold The on-manifold estimate containing this variable.
   * @return The @ref SE2TangentDistribution_ for this variable, or nullptr.
   */
  [[nodiscard]] std::shared_ptr<const SE2TangentDistribution_<T>> GetDistribution(
      const OnManifoldEstimate_<T>& estimate) const {
    auto marginal_estimate = estimate.Marginalize(this->Id());
    if (!marginal_estimate) {
      return nullptr;
    }
    auto marginal_distribution = marginal_estimate->GetDistribution();
    auto marginal_manifold = marginal_distribution->GetManifold();
    BELUGA_ESTIMATION_ASSERT(
        marginal_manifold->AmbientSize() == 4, "marginalized manifold ambient size does not match SE(2)");

    beluga_estimation::SE2<T> origin;
    marginal_distribution->GetOrigin(to_view(origin));
    return std::make_shared<SE2TangentDistribution_<T>>(marginal_distribution->GetEmbeddedDistribution(), origin);
  }
};

using SE2Variable = SE2Variable_<DefaultScalar>;

}  // namespace beluga_estimation

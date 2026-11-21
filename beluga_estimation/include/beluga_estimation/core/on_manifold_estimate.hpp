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
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/estimate.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/core/variable.hpp>

namespace beluga_estimation {

/**
 * @brief A multi-variable estimate over the tangent space of a manifold.
 *
 * An OnManifoldEstimate covers a set of variables whose combined state lives
 * on a manifold, with uncertainty represented by a @ref TangentDistribution_
 * over its tangent space. The tangent space of the manifold is the
 * concatenation of the individual variables' tangent spaces, in the same
 * order as the variables.
 *
 * The manifold is obtained from the @ref TangentDistribution_ itself, avoiding
 * duplication between the estimate and the distribution.
 *
 * Marginalization (@ref Marginalize) extracts per-variable or multi-variable
 * sub-estimates by building tangent-space slices from each variable's manifold
 * and delegating to @ref TangentDistribution_::Marginalize.
 *
 * @tparam T Scalar type (e.g. `float` or `double`).
 */
template <typename T>
class OnManifoldEstimate_ : public Estimate_<T> {
 public:
  /**
   * @brief Constructs an on-manifold estimate.
   *
   * @param variables Ordered list of variables, one per variable in the
   *   combined state. The tangent space of the distribution's manifold is
   *   the concatenation of each variable's manifold tangent space, in the
   *   same order.
   * @param distribution The tangent-space distribution over the combined
   *   manifold's tangent space.
   */
  OnManifoldEstimate_(
      std::vector<std::shared_ptr<const Variable_<T>>> variables,
      std::shared_ptr<const TangentDistribution_<T>> distribution)
      : variables_(std::move(variables)), distribution_(std::move(distribution)) {
    slices_.reserve(variables_.size());
    std::size_t offset = 0;
    for (std::size_t i = 0; i < variables_.size(); ++i) {
      auto manifold = variables_[i]->Manifold();
      auto [_, inserted] = index_.insert_or_assign(variables_[i]->Id(), i);
      BELUGA_ESTIMATION_DEMAND(inserted, "duplicate variable IDs are not allowed");
      slices_.push_back(contiguous_slice::Span(offset, manifold->TangentSize()));
      offset += manifold->TangentSize();
    }
    BELUGA_ESTIMATION_DEMAND(
        offset == distribution_->GetManifold()->TangentSize(),
        "tangent space of all variables combined must match that of the distribution");
  }

  /// @brief Returns the variables covered by this estimate.
  beluga_estimation::view<std::shared_ptr<const Variable_<T>>> GetVariables() const noexcept override {
    return variables_;
  }

  /// @brief Returns the typed tangent-space distribution.
  [[nodiscard]] std::shared_ptr<const TangentDistribution_<T>> GetDistribution() const noexcept {
    return distribution_;
  }

  /**
   * @brief Marginalizes this estimate to a subset of variables.
   *
   * @param variable_ids The subset of variables to marginalize to, identified
   *   by their @ref Identifier. Must be a non-empty, strict subset of this
   *   estimate's variable set.
   * @return A shared pointer to the marginalized OnManifoldEstimate_, or
   *   nullptr if marginalization fails.
   */
  [[nodiscard]] std::shared_ptr<OnManifoldEstimate_<T>> Marginalize(
      std_future::span<const Identifier> variable_ids) const {
    if (variable_ids.empty()) {
      return nullptr;
    }
    std::vector<contiguous_slice> slices;
    std::vector<std::shared_ptr<const Variable_<T>>> marginalized_variables;
    slices.reserve(variable_ids.size());
    marginalized_variables.reserve(variable_ids.size());
    for (const Identifier& variable_id : variable_ids) {
      if (auto it = index_.find(variable_id); it != index_.end()) {
        slices.push_back(slices_[it->second]);
        marginalized_variables.push_back(variables_[it->second]);
      } else {
        return nullptr;
      }
    }

    // Marginalize the distribution.
    auto marginal_distribution = distribution_->Marginalize(slices);
    if (!marginal_distribution) {
      return nullptr;
    }

    return std::make_shared<OnManifoldEstimate_<T>>(
        std::move(marginalized_variables), std::move(marginal_distribution));
  }

  /// @brief Marginalizes this estimate to a single variable.
  ///
  /// Convenience overload of @ref Marginalize(span) that avoids allocating a
  /// temporary container for a single identifier.
  ///
  /// @param variable_id The identifier of the variable to marginalize to.
  /// @return A shared pointer to the marginalized OnManifoldEstimate_, or
  ///   nullptr if marginalization fails.
  [[nodiscard]] std::shared_ptr<OnManifoldEstimate_<T>> Marginalize(const Identifier& variable_id) const {
    return Marginalize(std_future::span<const Identifier>{&variable_id, 1});
  }

 protected:
  std::shared_ptr<Estimate_<T>> DoMarginalize(std_future::span<const Identifier> variable_ids) const override {
    return Marginalize(variable_ids);
  }

  std::shared_ptr<const Distribution_<T>> DoGetDistribution() const noexcept override { return GetDistribution(); }

 private:
  std::vector<contiguous_slice> slices_;
  std::unordered_map<Identifier, std::size_t> index_;
  std::vector<std::shared_ptr<const Variable_<T>>> variables_;
  std::shared_ptr<const TangentDistribution_<T>> distribution_;
};

/// @brief Default-scalar alias for OnManifoldEstimate_.
using OnManifoldEstimate = OnManifoldEstimate_<DefaultScalar>;

}  // namespace beluga_estimation

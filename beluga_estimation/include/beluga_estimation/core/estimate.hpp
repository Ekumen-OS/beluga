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

#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/distribution.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/core/variable.hpp>

namespace beluga_estimation {

/// @brief Base interface for a probabilistic estimate over a set of variables.
///
/// An estimate associates a set of variables with a manifold and a
/// distribution. The distribution lives either in the tangent space of the
/// manifold (for tangent-space estimates) or in ambient space (for
/// specialized estimates such as directional distributions).
///
/// Marginalization may or may not be supported by a concrete implementation.
/// Call @ref Marginalize() and check whether the result is non-null.
/// Single-variable estimates returned by this method cannot be further
/// marginalized (subsequent calls return nullptr).
///
/// @tparam T Scalar type (e.g. `float` or `double`).
template <typename T>
class Estimate_ {
 public:
  virtual ~Estimate_() = default;

  /// @brief Returns the variables covered by this estimate.
  /// @return A view over the ordered list of variables.
  [[nodiscard]] virtual beluga_estimation::view<std::shared_ptr<const Variable_<T>>> GetVariables() const noexcept = 0;

  /// @brief Returns the probability distribution for this estimate.
  ///
  /// The distribution is defined over the tangent space of the manifold
  /// (for tangent-space estimates) or over ambient space (for specialized cases).
  /// @return A shared pointer to the distribution.
  [[nodiscard]] std::shared_ptr<const Distribution_<T>> GetDistribution() const noexcept {
    return DoGetDistribution();
  };

  /// @brief Marginalizes this estimate to a subset of variables.
  ///
  /// Not all estimate types support marginalization. Implementations that do
  ///   (e.g. @ref OnManifoldEstimate_) return a new estimate over the requested
  /// variable subset. Implementations that do not return nullptr.
  ///
  /// When the subset is a single variable, the result is a base @ref Estimate_
  /// that does not support further marginalization (subsequent calls return
  /// nullptr).
  ///
  /// @param variable_ids The subset of variables to marginalize to, identified
  ///   by their @ref Identifier. Must be a non-empty, strict subset of this
  ///   estimate's variable set.
  /// @return A shared pointer to the marginalized estimate, or nullptr if
  ///   marginalization is not supported, the subset is invalid, or any
  ///   variable is not found in this estimate's variable set.
  std::shared_ptr<Estimate_<T>> Marginalize(std_future::span<const Identifier> variable_ids) const {
    return DoMarginalize(variable_ids);
  };

  /// @brief Marginalizes this estimate to a single variable.
  ///
  /// Convenience overload of @ref Marginalize(span) that avoids allocating a
  /// temporary container for a single identifier.
  ///
  /// @param variable_id The identifier of the variable to marginalize to.
  /// @return A shared pointer to the marginalized estimate, or nullptr if
  ///   marginalization is not supported, the identifier is not found, or the
  ///   subset is invalid.
  std::shared_ptr<Estimate_<T>> Marginalize(const Identifier& variable_id) const {
    return Marginalize(std_future::span<const Identifier>{&variable_id, 1});
  }

 protected:
  [[nodiscard]] virtual std::shared_ptr<Estimate_<T>> DoMarginalize(
      std_future::span<const Identifier> variable_ids) const = 0;

  /// @brief Returns the probability distribution for this estimate.
  ///
  /// This is a protected virtual method to allow derived classes to override
  /// the distribution type (e.g., @ref TangentDistribution_).
  [[nodiscard]] virtual std::shared_ptr<const Distribution_<T>> DoGetDistribution() const noexcept = 0;
};

using Estimate = Estimate_<DefaultScalar>;

}  // namespace beluga_estimation

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
#include <cstddef>
#include <memory>
#include <utility>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/core/estimate.hpp>
#include <beluga_estimation/core/on_manifold_estimate.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/core/variable.hpp>
#include <beluga_estimation/manifolds/euclidean_manifold.hpp>

namespace beluga_estimation {

/**
 * @brief A variable that belongs to a Euclidean vector space.
 *
 * Does not store a distribution directly — the distribution is extracted
 * from an @ref Estimate_ via @ref GetDistribution.
 *
 * Euclidean variables are always dynamically dimensioned: the dimension is a
 * property of the schema and the configuration, not of the C++ type. Callers
 * that know their own dimensionality at compile time can say so by constructing
 * with an @ref NDim tag, which binds the statically sized manifold singleton
 * instead of allocating one, but that is an implementation detail of this
 * variable — see @ref Size for the authoritative dimension.
 *
 * @tparam T The numeric scalar type.
 */
template <typename T>
class EuclideanVariable_ : public Variable_<T> {
 public:
  /**
   * @brief Constructs a Euclidean variable of runtime dimension.
   *
   * Allocates a dynamically sized manifold. Use this overload when the
   * dimension only becomes known at runtime (e.g. from a parameter vector).
   *
   * @param id The unique identifier for this variable node.
   * @param ndim The dimension of the variable. Must be greater than zero.
   */
  EuclideanVariable_(Identifier id, std::size_t ndim)
      : Variable_<T>(std::move(id)), manifold_{std::make_shared<EuclideanManifold_<T>>(ndim)} {
    BELUGA_ESTIMATION_DEMAND(ndim > 0, "dimension must be > 0");
  }

  /**
   * @brief Constructs a Euclidean variable of compile-time dimension.
   *
   * Binds the @ref EuclideanManifold_ singleton for @p N, so no manifold is
   * allocated. The variable itself remains dynamically dimensioned as far as
   * the network is concerned; the static size is recoverable only through the
   * manifold, never through the variable's type.
   *
   * @param id The unique identifier for this variable node.
   * @param ndim The dimension tag. Must be greater than zero.
   */
  template <std::size_t N>
  EuclideanVariable_(Identifier id, NDim<N> ndim)
      : Variable_<T>(std::move(id)), manifold_{EuclideanManifold_<T, N>::Instance()} {
    static_assert(N > 0, "dimension must be > 0");
    (void)ndim;
  }

  [[nodiscard]] std::shared_ptr<const beluga_estimation::Manifold_<T>> Manifold() const override { return manifold_; }

  /**
   * @brief Returns the dimension of this variable.
   *
   * This is the only dimensional guarantee the variable offers. Code that
   * pulls a variable from the network must compare @ref Size against its own
   * expectations before taking a fixed-extent view over the variable's value.
   *
   * @return The ambient dimension of this variable's manifold.
   */
  [[nodiscard]] std::size_t Size() const { return Manifold()->AmbientSize(); }

  using Variable_<T>::GetDistribution;

  /**
   * @brief Extracts the Euclidean distribution from an @ref OnManifoldEstimate_
   *        (typed getter).
   *
   * For Euclidean variables the origin is implicitly zero, so the tangent-space
   * Euclidean distribution represents the variable's distribution directly.
   *
   * @param on_manifold The on-manifold estimate containing this variable.
   * @return The @ref EuclideanDistribution_ for this variable, or nullptr.
   */
  [[nodiscard]] std::shared_ptr<const beluga_estimation::EuclideanDistribution_<T>> GetDistribution(
      const OnManifoldEstimate_<T>& estimate) const {
    auto marginal_estimate = estimate.Marginalize(this->Id());
    if (!marginal_estimate) {
      return nullptr;
    }
    auto marginal_distribution = marginal_estimate->GetDistribution();
    return marginal_distribution->GetEmbeddedDistribution();
  }

 private:
  std::shared_ptr<const beluga_estimation::Manifold_<T>> manifold_;
};

using EuclideanVariable = EuclideanVariable_<DefaultScalar>;

}  // namespace beluga_estimation

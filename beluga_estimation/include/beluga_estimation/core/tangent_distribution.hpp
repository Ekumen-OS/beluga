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
#include <vector>

#include <range/v3/range/primitives.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/distribution.hpp>
#include <beluga_estimation/core/euclidean_distribution.hpp>
#include <beluga_estimation/core/manifold.hpp>

namespace beluga_estimation {

/**
 * @brief Interface for distributions over the tangent space of a manifold.
 *
 * A tangent-space distribution is defined by three components:
 *   1. A **manifold** whose tangent space this distribution lives in.
 *   2. An **origin** (linearization point) on the manifold's ambient space.
 *   3. A **Euclidean distribution** over the tangent vector space at the origin.
 *
 * The ambient-space interface inherited from @ref Distribution_<T> (e.g.,
 * @ref LogProbability, @ref Sample) operates by lifting/lowering through
 * the manifold's Plus/Minus operations relative to the origin.
 *
 * The tangent-space interface (e.g., @ref ComputeCovariance) delegates to
 * the inner Euclidean distribution.
 *
 * Concrete implementations must provide @ref GetManifold,
 * @ref GetEmbeddedDistribution, and @ref GetOrigin.
 *
 * @tparam T Scalar type (e.g. `float` or `double`).
 */
template <typename T>
class TangentDistribution_ : public Distribution_<T> {
 public:
  /// @brief Returns the dimensionality of the tangent-space distribution domain.
  ///
  /// The domain size corresponds to the manifold's ambient dimension.
  [[nodiscard]] std::size_t DomainSize() const override { return GetManifold()->AmbientSize(); }

  /// @brief Returns the manifold.
  [[nodiscard]] virtual std::shared_ptr<const Manifold_<T>> GetManifold() const = 0;

  /// @brief Returns the inner Euclidean distribution over the tangent space.
  [[nodiscard]] virtual std::shared_ptr<const EuclideanDistribution_<T>> GetEmbeddedDistribution() const = 0;

  /// @brief Reads the origin into the output span.
  virtual void GetOrigin(std_future::span<T> origin) const = 0;

  /**
   * @brief Computes the ambient-space mean by retracting the origin by the
   *        tangent-space mean.
   *
   * The algorithm is:
   *   1. Retrieve the origin (linearization point) via @ref GetOrigin.
   *   2. Compute the mean of the inner tangent-space distribution via
   *      @ref EuclideanDistribution_::ComputeMean.
   *   3. Retract via the manifold's Plus operation:
   *      $\text{mean} = \text{origin} \oplus \overline{\delta}$.
   */
  void ComputeMean(std_future::span<T> mean) const override {
    auto manifold = GetManifold();
    auto embedded_distribution = GetEmbeddedDistribution();

    beluga_estimation::VectorX<T> origin(manifold->AmbientSize());
    GetOrigin(to_view(origin));

    beluga_estimation::VectorX<T> embedded_mean(embedded_distribution->DomainSize());
    embedded_distribution->ComputeMean(to_view(embedded_mean));

    manifold->Plus(to_view(origin), to_view(embedded_mean), mean);
  }

  /**
   * @brief Computes the covariance matrix in the tangent space.
   *
   * Delegates directly to the inner Euclidean distribution (covariance is
   * always defined in the tangent space).
   */
  void ComputeCovariance(std_future::mdspan<T, 2> covariance) const override {
    GetEmbeddedDistribution()->ComputeCovariance(covariance);
  }

  /**
   * @brief Computes the log-probability of an ambient-space state.
   *
   * The algorithm is:
   *   1. Retrieve the origin (linearization point) via @ref GetOrigin.
   *   2. Compute the tangent-space residual via the manifold's Minus operation:
   *      $\delta = \text{value} \ominus \text{origin}$.
   *   3. Evaluate the inner Euclidean log-PDF at $\delta$.
   */
  T LogProbability(std_future::span<const T> value) const override {
    auto manifold = GetManifold();
    auto embedded_distribution = GetEmbeddedDistribution();

    beluga_estimation::VectorX<T> origin(manifold->AmbientSize());
    GetOrigin(to_view(origin));

    beluga_estimation::VectorX<T> residual(embedded_distribution->DomainSize());
    manifold->Minus(value, to_view(origin), to_view(residual));

    return embedded_distribution->LogProbability(to_view(residual));
  }

  /**
   * @brief Generates ambient-space samples.
   *
   * The algorithm is:
   *   1. Retrieve the origin (linearization point) via @ref GetOrigin.
   *   2. Draw samples from the inner tangent-space distribution via
   *      @ref EuclideanDistribution_::Sample.
   *   3. Retract each tangent-space sample via the manifold's Plus operation:
   *      $\text{sample}_i = \text{origin} \oplus \delta_i$.
   */
  void Sample(beluga_estimation::view<std_future::span<T>> values, RandomNumberGenerator& gen) const override {
    auto manifold = GetManifold();
    auto embedded_distribution = GetEmbeddedDistribution();

    beluga_estimation::VectorX<T> origin(manifold->AmbientSize());
    GetOrigin(to_view(origin));

    std::vector<beluga_estimation::VectorX<T>> tangent_samples(
        ranges::size(values), beluga_estimation::VectorX<T>(embedded_distribution->DomainSize()));
    embedded_distribution->Sample(beluga_estimation::to_view(tangent_samples), gen);

    for (size_t i = 0; i < ranges::size(values); ++i) {
      manifold->Plus(to_view(origin), to_view(tangent_samples[i]), values[i]);
    }
  }

  /**
   * @brief Marginalizes this distribution to a subset of tangent-space dimensions.
   *
   * Each slice in @p dims selects a set of contiguous dimension indices to retain.
   * The slices must not overlap.
   *
   * @param dims Slice specifications describing the subset of dimensions to retain.
   * @return A shared pointer to the marginalized distribution, or nullptr
   *   if marginalization is not supported.
   */
  [[nodiscard]] std::shared_ptr<TangentDistribution_<T>> Marginalize(
      beluga_estimation::view<const contiguous_slice> dims) const {
    return std::static_pointer_cast<TangentDistribution_<T>>(this->DoMarginalize(dims));
  }

  /**
   * @brief Marginalizes this distribution to a single contiguous block.
   * @copydetails TangentDistribution_::Marginalize
   */
  [[nodiscard]] std::shared_ptr<TangentDistribution_<T>> Marginalize(const contiguous_slice& dims) const {
    return Marginalize(ranges::views::single(dims));
  }
};

/// @brief Default-scalar alias for TangentDistribution_.
using TangentDistribution = TangentDistribution_<DefaultScalar>;

}  // namespace beluga_estimation

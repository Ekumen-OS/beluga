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

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/euclidean_distribution.hpp>
#include <beluga_estimation/core/manifold.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief A concrete tangent-space distribution that stores its state as vectors.
 *
 * This is the default concrete implementation of @ref TangentDistribution_ for
 * manifolds whose ambient and tangent spaces are plain vectors (e.g., Euclidean
 * manifolds). It stores:
 *   - A shared pointer to the manifold.
 *   - A shared pointer to the inner Euclidean distribution.
 *   - The origin as a vector of size `AmbientSize()`.
 *
 * Marginalization is supported: the origin is projected to the sub-manifold,
 * the embedded Euclidean distribution is marginalized, and a new
 * @ref GenericTangentDistribution_ wrapping the results is returned.
 *
 * @tparam T Scalar type (e.g., `float` or `double`).
 */
template <typename T>
class GenericTangentDistribution_ : public TangentDistribution_<T> {
 public:
  /**
   * @brief Constructs a generic tangent-space distribution.
   *
   * @param manifold The manifold whose tangent space this distribution lives in.
   * @param embedded_distribution The Euclidean distribution over the tangent
   *   space. Must have `Size()` equal to the manifold's `TangentSize()`.
   * @param origin The origin (linearization point) as a vector of `AmbientSize()`
   *   scalars.
   */
  GenericTangentDistribution_(
      std::shared_ptr<const Manifold_<T>> manifold,
      std::shared_ptr<const EuclideanDistribution_<T>> embedded_distribution,
      beluga_estimation::VectorX<T> origin)
      : manifold_(std::move(manifold)),
        embedded_distribution_(std::move(embedded_distribution)),
        origin_(std::move(origin)) {
    BELUGA_ESTIMATION_DEMAND(
        origin_.size() == static_cast<Eigen::Index>(manifold_->AmbientSize()),
        "origin size must match the manifold ambient size");
    BELUGA_ESTIMATION_DEMAND(
        embedded_distribution_->DomainSize() == manifold_->TangentSize(),
        "embedded distribution size must match the manifold tangent size");
  }

  /**
   * @brief Constructs a generic tangent-space distribution at the origin.
   *
   * @sa GenericTangentDistribution_(std::shared_ptr<const Manifold_<T>>, std::shared_ptr<const
   * EuclideanDistribution_<T>>, const beluga_estimation::VectorX<T>&)
   */
  GenericTangentDistribution_(
      std::shared_ptr<const Manifold_<T>> manifold,
      std::shared_ptr<const EuclideanDistribution_<T>> embedded_distribution)
      : GenericTangentDistribution_(
            manifold,
            std::move(embedded_distribution),
            beluga_estimation::VectorX<T>::Zero(manifold->AmbientSize())) {}

  /// @brief Returns the manifold.
  [[nodiscard]] std::shared_ptr<const Manifold_<T>> GetManifold() const override { return manifold_; }

  /// @brief Returns the inner Euclidean distribution over the tangent space.
  [[nodiscard]] std::shared_ptr<const EuclideanDistribution_<T>> GetEmbeddedDistribution() const override {
    return embedded_distribution_;
  }

  /// @brief Returns the origin (linearization point).
  [[nodiscard]] const beluga_estimation::VectorX<T>& Origin() const { return origin_; }

  /// @brief Copies the stored origin into the output span.
  void GetOrigin(std_future::span<T> origin) const override {
    GetOrigin(
        beluga_estimation::Map<beluga_estimation::VectorX<T>>(origin.data(), static_cast<Eigen::Index>(origin.size())));
  }

  /// @brief Copies the stored origin into the output Eigen vector.
  void GetOrigin(beluga_estimation::Ref<beluga_estimation::VectorX<T>> origin) const { origin = origin_; }

 protected:
  /**
   * @brief Marginalizes this distribution to a subset of tangent-space dimensions.
   *
   * Extracts the origin into the sub-manifold, marginalizes the embedded Euclidean
   * distribution, and returns a new @ref GenericTangentDistribution_ wrapping
   * the result.
   */
  [[nodiscard]] std::shared_ptr<Distribution_<T>> DoMarginalize(
      beluga_estimation::view<const contiguous_slice> dims) const override {
    if (!disjoint(dims)) {
      return nullptr;
    }

    // Compute total output dimension.
    std::size_t marginal_ndim = 0;
    for (const auto& dim : dims) {
      const std::size_t start = dim.start.value_or(0);
      const std::size_t end = dim.end.value_or(embedded_distribution_->DomainSize());
      marginal_ndim += end - start;
    }
    if (marginal_ndim == 0) {
      return nullptr;
    }

    // Get sub-manifold for this slice selection.
    auto submanifold = manifold_->GetSubManifold(dims);
    if (!submanifold) {
      return nullptr;
    }

    // Marginalize the tangent-space distribution.
    auto marginal_distribution = embedded_distribution_->Marginalize(dims);
    if (!marginal_distribution) {
      return nullptr;
    }

    // Extract the origin into the sub-manifold's ambient space.
    beluga_estimation::VectorX<T> marginal_origin(submanifold->AmbientSize());
    manifold_->EmbedAmbient(beluga_estimation::to_view(origin_), dims, beluga_estimation::to_view(marginal_origin));
    return std::make_shared<GenericTangentDistribution_<T>>(
        std::move(submanifold), std::move(marginal_distribution), marginal_origin);
  }

 private:
  std::shared_ptr<const Manifold_<T>> manifold_;
  std::shared_ptr<const EuclideanDistribution_<T>> embedded_distribution_;
  beluga_estimation::VectorX<T> origin_;
};

/// @brief Default-scalar alias for GenericTangentDistribution_.
using GenericTangentDistribution = GenericTangentDistribution_<DefaultScalar>;

}  // namespace beluga_estimation

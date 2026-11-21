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
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <range/v3/view/single.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/manifolds/se2_manifold.hpp>

namespace beluga_estimation {

/**
 * @brief SE(2)-specialized tangent-space distribution.
 *
 * A concrete implementation of @ref TangentDistribution_ for the SE(2)
 * manifold. The distribution lives in the 3D tangent space (vx, vy, ω)
 * with the origin being a point on the SE(2) manifold (a pose).
 *
 * The tangent-space component is a @ref EuclideanDistribution_ of size 3
 * (typically a @ref NormalDistribution_ or @ref DiracMixtureDistribution_).
 *
 * @tparam T Scalar type (e.g., double, float).
 */
template <typename T>
class SE2TangentDistribution_ : public TangentDistribution_<T> {
 public:
  /**
   * @brief Constructs an SE(2) tangent-space distribution.
   *
   * @param embedded_distribution A 3-dimensional Euclidean distribution over
   *   the tangent space (vx, vy, ω). Must have Size() == 3.
   * @param origin The SE(2) pose serving as the linearization point.
   */
  explicit SE2TangentDistribution_(
      std::shared_ptr<const EuclideanDistribution_<T>> embedded_distribution,
      const SE2<T>& origin = SE2<T>())
      : embedded_distribution_(std::move(embedded_distribution)), origin_(origin) {
    BELUGA_ESTIMATION_DEMAND(
        embedded_distribution_->DomainSize() == 3, "embedded distribution must have size 3 for SE(2)");
  }

  /// @brief Returns the SE(2) manifold.
  [[nodiscard]] std::shared_ptr<const Manifold_<T>> GetManifold() const override { return SE2Manifold_<T>::Instance(); }

  /// @brief Returns the embedded tangent-space distribution.
  [[nodiscard]] std::shared_ptr<const EuclideanDistribution_<T>> GetEmbeddedDistribution() const override {
    return embedded_distribution_;
  }

  /// @brief Copies the SE(2) origin into the output span (size 4).
  void GetOrigin(std_future::span<T> origin) const override {
    BELUGA_ESTIMATION_DEMAND(origin.size() == 4, "origin span must have size 4 for SE(2)");
    beluga_estimation::Map<SE2<T>>(origin.data()) = this->origin_;
  }

  /// @brief Returns the SE(2) origin (linearization point).
  [[nodiscard]] const SE2<T>& Origin() const { return origin_; }

  // Bring base class span-based overloads into overload set
  using TangentDistribution_<T>::ComputeMean;
  using TangentDistribution_<T>::ComputeCovariance;
  using TangentDistribution_<T>::LogProbability;
  using TangentDistribution_<T>::Sample;

  /**
   * @brief Computes the ambient-space SE(2) mean.
   * @param mean Output SE(2) pose to store the mean.
   */
  void ComputeMean(Ref<SE2<T>> mean) const { this->ComputeMean(to_view(mean)); }

  /**
   * @brief Computes the 3×3 tangent-space covariance matrix.
   * @param covariance Output matrix to store the covariance.
   */
  void ComputeCovariance(Ref<Matrix3<T>> covariance) const { this->ComputeCovariance(to_view(covariance)); }

  /// @brief Log-probability for SE(2) values.
  T LogProbability(Ref<SE2<T>> value) const { return this->LogProbability(to_view(value)); }

  /// @brief Sample into a view of SE(2) references.
  void Sample(view<Ref<SE2<T>>> values, RandomNumberGenerator& gen) const { Sample(to_view(values), gen); }

  /**
   * @brief Sample (convenience overload).
   * @param count Number of samples.
   * @param gen Random number generator.
   * @return A vector of SE(2) samples.
   */
  std::vector<SE2<T>> Sample(std::size_t count, RandomNumberGenerator& gen) const {
    std::vector<SE2<T>> samples(count);
    Sample(to_view(samples), gen);
    return samples;
  }

  /**
   * @brief Marginalizes this distribution to a subset of tangent-space dimensions.
   *
   * SE(2) only supports full marginalization (all 3 tangent dimensions).
   * Partial marginalization is not supported and returns nullptr.
   *
   * @param dims Slice specifications describing the subset of dimensions to retain.
   * @return A shared pointer to the marginalized distribution, or nullptr
   *   if the marginalization is not supported.
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

    // SE(2) only supports full marginalization.
    if (marginal_ndim != embedded_distribution_->DomainSize()) {
      return nullptr;
    }

    return std::make_shared<SE2TangentDistribution_<T>>(embedded_distribution_, origin_);
  }

 private:
  std::shared_ptr<const EuclideanDistribution_<T>> embedded_distribution_;
  SE2<T> origin_;
};

/// @brief Default-scalar alias for SE2TangentDistribution_.
using SE2TangentDistribution = SE2TangentDistribution_<DefaultScalar>;

}  // namespace beluga_estimation

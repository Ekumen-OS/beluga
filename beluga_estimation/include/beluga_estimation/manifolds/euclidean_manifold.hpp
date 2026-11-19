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
#include <optional>
#include <vector>

#include <Eigen/Core>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/manifold.hpp>

namespace beluga_estimation {

/**
 * @brief Manifold for N-dimensional Euclidean vector spaces (static size).
 *
 * The ambient and tangent spaces are both R^N. The @ref Plus and @ref Minus
 * operations correspond to standard vector addition and subtraction.
 *
 * @tparam T Numeric type used for scalar quantities.
 * @tparam N Dimension of the Euclidean space. Must be greater than zero.
 */
template <typename T, std::size_t N = 0>
class EuclideanManifold_ : public Manifold_<T> {
  static_assert(N > 0, "EuclideanManifold requires N > 0.");

 public:
  using Manifold_<T>::AmbientSize;
  using Manifold_<T>::GetSubManifold;
  using Manifold_<T>::LiftAmbient;
  using Manifold_<T>::EmbedAmbient;
  /// @brief Returns the ambient space dimension.
  [[nodiscard]] std::size_t AmbientSize() const override { return N; }

  /// @brief Returns the ambient size of the sub-manifold defined by tangent slices.
  ///
  /// For Euclidean spaces the sub-manifold ambient is the selected coordinates,
  /// so the size is the total number of selected tangent dimensions.
  ///
  /// @copydetails Manifold_::AmbientSize(view)
  [[nodiscard]] std::size_t AmbientSize(beluga_estimation::view<const contiguous_slice> tangent_dims) const override {
    Manifold_<T>::ValidateTangentDims(tangent_dims, N);
    std::size_t total = 0;
    for (const auto& dim : tangent_dims) {
      const std::size_t start = dim.start.value_or(0);
      const std::size_t end = dim.end.value_or(N);
      total += end - start;
    }
    return total;
  }

  /// @brief Returns the tangent space dimension.
  [[nodiscard]] std::size_t TangentSize() const override { return N; }

  /**
   * @brief Standard vector addition.
   * @copydetails Manifold_::Plus
   */
  void Plus(
      std_future::span<const T> lhs,
      std_future::span<const T> rhs,
      std_future::span<T> result,
      std::optional<std_future::mdspan<T, 2>> /*jacobian*/,
      std::optional<std_future::mdspan<T, 2>> /*hessian*/) const override {
    BELUGA_ESTIMATION_ASSERT(lhs.size() == N);
    BELUGA_ESTIMATION_ASSERT(rhs.size() == N);
    BELUGA_ESTIMATION_ASSERT(result.size() == N);

    beluga_estimation::Map<const beluga_estimation::Vector<T, N>> lhs_map(lhs.data());
    beluga_estimation::Map<const beluga_estimation::Vector<T, N>> rhs_map(rhs.data());
    beluga_estimation::Map<beluga_estimation::Vector<T, N>> result_map(result.data());

    result_map = lhs_map + rhs_map;
  }

  /**
   * @brief Standard vector subtraction.
   * @copydetails Manifold_::Minus
   */
  void Minus(
      std_future::span<const T> lhs,
      std_future::span<const T> rhs,
      std_future::span<T> result,
      std::optional<std_future::mdspan<T, 2>> /*jacobian*/,
      std::optional<std_future::mdspan<T, 2>> /*hessian*/) const override {
    BELUGA_ESTIMATION_ASSERT(lhs.size() == N);
    BELUGA_ESTIMATION_ASSERT(rhs.size() == N);
    BELUGA_ESTIMATION_ASSERT(result.size() == N);

    beluga_estimation::Map<const beluga_estimation::Vector<T, N>> lhs_map(lhs.data());
    beluga_estimation::Map<const beluga_estimation::Vector<T, N>> rhs_map(rhs.data());
    beluga_estimation::Map<beluga_estimation::Vector<T, N>> result_map(result.data());

    result_map = lhs_map - rhs_map;
  }

  /**
   * @brief Extracts a sub-manifold.
   * @copydetails Manifold_::GetSubManifold
   */
  [[nodiscard]] std::shared_ptr<const Manifold_<T>> GetSubManifold(
      beluga_estimation::view<const contiguous_slice> tangent_dims) const override {
    if (!disjoint(tangent_dims)) {
      return nullptr;
    }
    if (tangent_dims.size() == 0) {
      return nullptr;
    }
    std::size_t total_ndim = 0;
    for (const auto& dim : tangent_dims) {
      const std::size_t start = dim.start.value_or(0);
      const std::size_t end = dim.end.value_or(N);
      if (start > end || end > N) {
        return nullptr;
      }
      total_ndim += end - start;
    }
    if (total_ndim == 0) {
      return nullptr;
    }
    if (total_ndim == N) {
      return this->shared_from_this();
    }
    return std::make_shared<EuclideanManifold_<T>>(total_ndim);
  }

  /**
   * @brief Lifts a sub-manifold ambient vector into the full ambient space.
   *
   * For Euclidean spaces the ambient and tangent representations coincide and
   * the identity element is the zero vector, so this is equivalent to
   * @ref Manifold_::LiftTangent.
   *
   * @copydetails Manifold_::LiftAmbient
   */
  void LiftAmbient(
      std_future::span<const T> sub_ambient,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> full_ambient) const override {
    this->LiftTangent(sub_ambient, tangent_dims, full_ambient);
  }

  /**
   * @brief Embeds (extracts) a sub-manifold ambient vector from the full ambient space.
   *
   * For Euclidean spaces the ambient and tangent representations coincide, so
   * this is equivalent to @ref Manifold_::EmbedTangent.
   *
   * @copydetails Manifold_::EmbedAmbient
   */
  void EmbedAmbient(
      std_future::span<const T> full_ambient,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> sub_ambient) const override {
    this->EmbedTangent(full_ambient, tangent_dims, sub_ambient);
  }

  /**
   * @brief Gets the singleton instance.
   * @return A shared pointer to a statically-allocated instance.
   */
  [[nodiscard]] static std::shared_ptr<const EuclideanManifold_<T, N>> Instance() {
    static std::shared_ptr<EuclideanManifold_<T, N>> instance{new EuclideanManifold_<T, N>()};
    return instance;
  }

 private:
  EuclideanManifold_() = default;
};

/**
 * @brief Manifold for an N-dimensional Euclidean vector space (dynamic size).
 *
 * Specialization of @ref EuclideanManifold_ for runtime-sized spaces.
 * Useful for sub-manifold extraction.
 *
 * @tparam T Numeric type used for scalar quantities.
 */
template <typename T>
class EuclideanManifold_<T, 0> : public Manifold_<T> {
 public:
  using Manifold_<T>::AmbientSize;
  using Manifold_<T>::LiftAmbient;
  using Manifold_<T>::EmbedAmbient;

  /// @brief Constructs a dynamically-sized Euclidean manifold.
  explicit EuclideanManifold_(std::size_t ndim) : ndim_(ndim) {
    BELUGA_ESTIMATION_DEMAND(ndim > 0, "dimension must be > 0");
  }

  /// @brief Returns the ambient space dimension.
  [[nodiscard]] std::size_t AmbientSize() const override { return ndim_; }

  /// @brief Returns the ambient size of the sub-manifold defined by tangent slices.
  /// @copydetails EuclideanManifold_::AmbientSize(view)
  [[nodiscard]] std::size_t AmbientSize(beluga_estimation::view<const contiguous_slice> tangent_dims) const override {
    Manifold_<T>::ValidateTangentDims(tangent_dims, ndim_);
    std::size_t total = 0;
    for (const auto& dim : tangent_dims) {
      const std::size_t start = dim.start.value_or(0);
      const std::size_t end = dim.end.value_or(ndim_);
      total += end - start;
    }
    return total;
  }

  /// @brief Returns the tangent space dimension.
  [[nodiscard]] std::size_t TangentSize() const override { return ndim_; }

  /**
   * @brief Standard vector addition.
   * @copydetails Manifold_::Plus
   */
  void Plus(
      std_future::span<const T> lhs,
      std_future::span<const T> rhs,
      std_future::span<T> result,
      std::optional<std_future::mdspan<T, 2>> /*jacobian*/,
      std::optional<std_future::mdspan<T, 2>> /*hessian*/) const override {
    BELUGA_ESTIMATION_ASSERT(lhs.size() == ndim_);
    BELUGA_ESTIMATION_ASSERT(rhs.size() == ndim_);
    BELUGA_ESTIMATION_ASSERT(result.size() == ndim_);

    beluga_estimation::Map<const beluga_estimation::VectorX<T>> lhs_map(lhs.data(), static_cast<Eigen::Index>(ndim_));
    beluga_estimation::Map<const beluga_estimation::VectorX<T>> rhs_map(rhs.data(), static_cast<Eigen::Index>(ndim_));
    beluga_estimation::Map<beluga_estimation::VectorX<T>> result_map(result.data(), static_cast<Eigen::Index>(ndim_));

    result_map = lhs_map + rhs_map;
  }

  /**
   * @brief Standard vector subtraction.
   * @copydetails Manifold_::Minus
   */
  void Minus(
      std_future::span<const T> lhs,
      std_future::span<const T> rhs,
      std_future::span<T> result,
      std::optional<std_future::mdspan<T, 2>> /*jacobian*/,
      std::optional<std_future::mdspan<T, 2>> /*hessian*/) const override {
    BELUGA_ESTIMATION_ASSERT(lhs.size() == ndim_);
    BELUGA_ESTIMATION_ASSERT(rhs.size() == ndim_);
    BELUGA_ESTIMATION_ASSERT(result.size() == ndim_);

    beluga_estimation::Map<const beluga_estimation::VectorX<T>> lhs_map(lhs.data(), static_cast<Eigen::Index>(ndim_));
    beluga_estimation::Map<const beluga_estimation::VectorX<T>> rhs_map(rhs.data(), static_cast<Eigen::Index>(ndim_));
    beluga_estimation::Map<beluga_estimation::VectorX<T>> result_map(result.data(), static_cast<Eigen::Index>(ndim_));

    result_map = lhs_map - rhs_map;
  }

  /**
   * @brief Extracts a sub-manifold.
   * @copydetails Manifold_::GetSubManifold
   */
  [[nodiscard]] std::shared_ptr<const Manifold_<T>> GetSubManifold(
      beluga_estimation::view<const contiguous_slice> tangent_dims) const override {
    if (!disjoint(tangent_dims)) {
      return nullptr;
    }
    if (tangent_dims.size() == 0) {
      return nullptr;
    }
    std::size_t total_ndim = 0;
    for (const auto& dim : tangent_dims) {
      const std::size_t start = dim.start.value_or(0);
      const std::size_t end = dim.end.value_or(ndim_);
      if (start > end || end > ndim_) {
        return nullptr;
      }
      total_ndim += end - start;
    }
    if (total_ndim == 0) {
      return nullptr;
    }
    if (total_ndim == ndim_) {
      return this->shared_from_this();
    }
    return std::make_shared<EuclideanManifold_<T>>(total_ndim);
  }

  /**
   * @brief Lifts a sub-manifold ambient vector into the full ambient space.
   * @copydetails EuclideanManifold_::LiftAmbient
   */
  void LiftAmbient(
      std_future::span<const T> sub_ambient,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> full_ambient) const override {
    this->LiftTangent(sub_ambient, tangent_dims, full_ambient);
  }

  /**
   * @brief Embeds (extracts) a sub-manifold ambient vector from the full ambient space.
   * @copydetails EuclideanManifold_::EmbedAmbient
   */
  void EmbedAmbient(
      std_future::span<const T> full_ambient,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> sub_ambient) const override {
    this->EmbedTangent(full_ambient, tangent_dims, sub_ambient);
  }

 private:
  std::size_t ndim_ = 0;
};

using EuclideanManifold = EuclideanManifold_<DefaultScalar>;

using Euclidean2Manifold = EuclideanManifold_<DefaultScalar, 2>;

using Euclidean3Manifold = EuclideanManifold_<DefaultScalar, 3>;

template <std::size_t N>
using EuclideanNManifold = EuclideanManifold_<DefaultScalar, N>;

}  // namespace beluga_estimation

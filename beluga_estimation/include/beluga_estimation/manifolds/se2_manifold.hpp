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
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/manifold.hpp>

namespace beluga_estimation {

/**
 * @brief Manifold operations for the 2D Special Euclidean group $SE(2)$.
 *
 * This class provides the local retraction (Plus) and local coordinates (Minus) operations
 * for states on the $SE(2)$ manifold, commonly representing 2D poses $(x, y, \theta)$.
 *
 * The underlying ambient representation uses Sophus memory layout:
 * `[cos(theta), sin(theta), x, y]` (size 4).
 * The tangent space representation is the Lie algebra $\mathfrak{se}(2)$:
 * `[v_x, v_y, omega]` (size 3).
 *
 * @tparam T The scalar type (e.g., double, float).
 */
template <typename T>
class SE2Manifold_ : public Manifold_<T> {
 public:
  using Manifold_<T>::AmbientSize;
  using Manifold_<T>::GetSubManifold;
  using Manifold_<T>::LiftAmbient;
  using Manifold_<T>::EmbedAmbient;

  /**
   * @brief Returns the size of the ambient representation.
   * @return 4, corresponding to `[cos(theta), sin(theta), x, y]`.
   */
  std::size_t AmbientSize() const override { return 4; }

  /**
   * @brief Returns the ambient size of the sub-manifold defined by tangent slices.
   *
   * SE(2) does not support proper sub-manifolds: the only valid selection is
   * the full tangent space, whose ambient size is `AmbientSize()`.
   *
   * @copydetails Manifold_::AmbientSize(view)
   * @throws std::invalid_argument if @p tangent_dims do not select the full tangent space.
   */
  [[nodiscard]] std::size_t AmbientSize(beluga_estimation::view<const contiguous_slice> tangent_dims) const override {
    ValidateFullSelection(tangent_dims);
    return AmbientSize();
  }

  /**
   * @brief Returns the size of the tangent space.
   * @return 3, corresponding to the Lie algebra `[v_x, v_y, omega]`.
   */
  std::size_t TangentSize() const override { return 3; }

  /**
   * @brief Gets the singleton instance of this manifold.
   * @return A shared pointer to the shared static instance.
   */
  static std::shared_ptr<const SE2Manifold_<T>> Instance() {
    static std::shared_ptr<SE2Manifold_<T>> instance{new SE2Manifold_<T>()};
    return instance;
  }

  /**
   * @brief Retraction on SE(2) using the exponential map.
   * Computes `result = lhs * exp(rhs)` as a group multiplication.
   * @copydetails Manifold_::Plus
   */
  void Plus(
      std_future::span<const T> lhs,
      std_future::span<const T> rhs,
      std_future::span<T> result,
      std::optional<std_future::mdspan<T, 2>> /*jacobian*/,
      std::optional<std_future::mdspan<T, 2>> /*hessian*/) const override {
    BELUGA_ESTIMATION_ASSERT(lhs.size() == AmbientSize());
    BELUGA_ESTIMATION_ASSERT(rhs.size() == TangentSize());
    BELUGA_ESTIMATION_ASSERT(result.size() == AmbientSize());

    const auto lhs_se2 = beluga_estimation::Map<const beluga_estimation::SE2<T>>(lhs.data());
    const auto rhs_tangent = beluga_estimation::Map<const typename beluga_estimation::SE2<T>::Tangent>(rhs.data());
    auto result_se2 = beluga_estimation::Map<beluga_estimation::SE2<T>>(result.data());

    result_se2 = lhs_se2 * beluga_estimation::SE2<T>::exp(rhs_tangent);
  }

  /**
   * @brief Local coordinates on SE(2) using the logarithmic map.
   * Computes `result = log(rhs^{-1} * lhs)` as a group logarithm.
   * @copydetails Manifold_::Minus
   */
  void Minus(
      std_future::span<const T> lhs,
      std_future::span<const T> rhs,
      std_future::span<T> result,
      std::optional<std_future::mdspan<T, 2>> /*jacobian*/,
      std::optional<std_future::mdspan<T, 2>> /*hessian*/) const override {
    BELUGA_ESTIMATION_ASSERT(lhs.size() == AmbientSize());
    BELUGA_ESTIMATION_ASSERT(rhs.size() == AmbientSize());
    BELUGA_ESTIMATION_ASSERT(result.size() == TangentSize());

    const auto lhs_se2 = beluga_estimation::Map<const beluga_estimation::SE2<T>>(lhs.data());
    const auto rhs_se2 = beluga_estimation::Map<const beluga_estimation::SE2<T>>(rhs.data());
    auto result_tangent = beluga_estimation::Map<typename beluga_estimation::SE2<T>::Tangent>(result.data());
    result_tangent = (rhs_se2.inverse() * lhs_se2).log();
  }

  /**
   * @brief Lifts a sub-manifold ambient vector into the full ambient space.
   *
   * SE(2) does not support proper sub-manifolds: the only valid selection is
   * the full tangent space $[0, 3)$.
   *
   * @copydetails Manifold_::LiftAmbient
   * @throws std::invalid_argument if @p tangent_dims do not select the full tangent space.
   */
  void LiftAmbient(
      std_future::span<const T> sub_ambient,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> full_ambient) const override {
    ValidateFullSelection(tangent_dims);
    BELUGA_ESTIMATION_ASSERT(sub_ambient.size() == AmbientSize());
    BELUGA_ESTIMATION_ASSERT(full_ambient.size() == AmbientSize());
    std::copy_n(sub_ambient.begin(), full_ambient.size(), full_ambient.begin());
  }

  /**
   * @brief Embeds (extracts) a sub-manifold ambient vector from the full ambient space.
   *
   * SE(2) does not support proper sub-manifolds: the only valid selection is
   * the full tangent space $[0, 3)$.
   *
   * @copydetails Manifold_::EmbedAmbient
   * @throws std::invalid_argument if @p tangent_dims do not select the full tangent space.
   */
  void EmbedAmbient(
      std_future::span<const T> full_ambient,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> sub_ambient) const override {
    ValidateFullSelection(tangent_dims);
    BELUGA_ESTIMATION_ASSERT(full_ambient.size() == AmbientSize());
    BELUGA_ESTIMATION_ASSERT(sub_ambient.size() == AmbientSize());
    std::copy_n(full_ambient.begin(), sub_ambient.size(), sub_ambient.begin());
  }

  /**
   * @brief Partially supported for SE(2).
   */
  [[nodiscard]] std::shared_ptr<const Manifold_<T>> GetSubManifold(
      beluga_estimation::view<const contiguous_slice> dims) const override {
    if (dims.size() != 1 || dims[0].start.value_or(0) != 0 || dims[0].end.value_or(TangentSize()) != TangentSize()) {
      return nullptr;
    }
    return this->shared_from_this();
  }

 private:
  /// @brief Validates that the selection covers the full tangent space.
  /// @throws std::invalid_argument if @p dims do not select the full tangent space.
  void ValidateFullSelection(beluga_estimation::view<const contiguous_slice> dims) const {
    BELUGA_ESTIMATION_DEMAND(
        dims.size() == 1 && dims[0].start.value_or(0) == 0 && dims[0].end.value_or(TangentSize()) == TangentSize(),
        "SE(2) manifold does not support sub-manifold lift/embed");
  }

  SE2Manifold_() = default;
};

using SE2Manifold = SE2Manifold_<DefaultScalar>;

}  // namespace beluga_estimation

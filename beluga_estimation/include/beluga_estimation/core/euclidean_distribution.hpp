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

#include <Eigen/Core>
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/distribution.hpp>

namespace beluga_estimation {

/**
 * @brief Interface for probability distributions over R^N Euclidean space.
 * *
 * The typed virtual methods accept @c beluga_estimation::Ref<Vector<T, N>> for zero-copy
 * interop with both concrete Eigen vectors and @c beluga_estimation::Map views. Concrete
 * distributions implement these typed methods directly.
 *
 * @tparam T Scalar type (e.g. `float` or `double`).
 */
template <typename T>
class EuclideanDistribution_ : public Distribution_<T> {
 public:
  ~EuclideanDistribution_() override = default;

  // ------------------------------------------------------------------
  // Typed virtual methods (concrete distributions implement these)
  // ------------------------------------------------------------------

  /**
   * @brief Computes the mean of the distribution (typed interface).
   *
   * Concrete distributions implement this. Uses @c Eigen::Ref which
   * accepts both concrete vectors and @c Eigen::Map.
   *
   * @param mean An Eigen::Ref to an N-dimensional vector to store the mean.
   */
  virtual void ComputeMean(beluga_estimation::Ref<VectorX<T>> mean) const = 0;

  /**
   * @brief Computes the covariance matrix (typed interface).
   *
   * Concrete distributions implement this.
   *
   * @param cov An Eigen::Ref to an N×N matrix to store the covariance.
   */
  virtual void ComputeCovariance(beluga_estimation::Ref<MatrixX<T>> cov) const = 0;

  /**
   * @brief Computes the log-probability density at a given state (typed interface).
   *
   * Concrete distributions implement this.
   *
   * @param value An Eigen::Ref to an N-dimensional vector to evaluate.
   * @return The log-probability density.
   */
  virtual T LogProbability(beluga_estimation::Ref<const VectorX<T>> value) const = 0;

  /**
   * @brief Generates random samples (typed interface).
   *
   * Concrete distributions implement this. Each output element is an
   * @c beluga_estimation::Ref<Vector<T, N>> that writes into the caller's storage.
   *
   * @param values A view of @c beluga_estimation::Ref<Vector<T, N>> to store samples.
   * @param gen The random number generator to use.
   */
  virtual void Sample(beluga_estimation::view<beluga_estimation::Ref<VectorX<T>>> values, RandomNumberGenerator& gen)
      const = 0;

  /**
   * @brief Computes the mean of the distribution (span-based interface).
   *
   * Creates an @c beluga_estimation::Map from the span and delegates to
   * @ref ComputeMean(beluga_estimation::Ref<Vector<T, N>>).
   */
  void ComputeMean(std_future::span<T> mean) const override {
    BELUGA_ESTIMATION_ASSERT(mean.size() == this->DomainSize(), "mean span must have size N");
    this->ComputeMean(beluga_estimation::Map<VectorX<T>>(mean.data(), static_cast<Eigen::Index>(mean.size())));
  }

  /**
   * @brief Computes the covariance matrix (span-based interface).
   *
   * Creates a @c beluga_estimation::Map from the mdspan and delegates to
   * @ref ComputeCovariance(beluga_estimation::Ref<Matrix<T, N, N>>).
   */
  void ComputeCovariance(std_future::mdspan<T, 2> covariance) const override {
    BELUGA_ESTIMATION_ASSERT(covariance.extent(0) == this->DomainSize(), "covariance mdspan must have rows == N");
    BELUGA_ESTIMATION_ASSERT(covariance.extent(1) == this->DomainSize(), "covariance mdspan must have cols == N");
    this->ComputeCovariance(beluga_estimation::Map<MatrixX<T>>(covariance));
  }

  /**
   * @brief Computes the log-probability density at a given state (span-based interface).
   *
   * Creates an @c beluga_estimation::Map from the span and delegates to
   * @ref LogProbability(beluga_estimation::Ref<const Vector<T, N>>).
   */
  T LogProbability(std_future::span<const T> value) const override {
    BELUGA_ESTIMATION_ASSERT(value.size() == this->DomainSize(), "value span must have size N");
    return this->LogProbability(
        beluga_estimation::Map<const VectorX<T>>(value.data(), static_cast<Eigen::Index>(value.size())));
  }

  /**
   * @brief Generates random samples (span-based interface).
   *
   * Creates @c beluga_estimation::Ref<Vector<T, N>> from each span and delegates to
   * @ref Sample(beluga_estimation::view<beluga_estimation::Ref<Vector<T, N>>>, RandomNumberGenerator&).
   */
  void Sample(beluga_estimation::view<std_future::span<T>> values, RandomNumberGenerator& gen) const override {
    auto values_view = ranges::views::transform(values, [n = static_cast<Eigen::Index>(this->DomainSize())](auto span) {
      BELUGA_ESTIMATION_ASSERT(span.size() == n, "each sample span must have size N");
      return beluga_estimation::Ref<VectorX<T>>(beluga_estimation::Map<VectorX<T>>(span.data(), n));
    });
    this->Sample(values_view, gen);
  }

  /**
   * @brief Marginalizes this distribution to a subset of dimensions.
   * @copydetails Distribution_::Marginalize
   */
  [[nodiscard]] std::shared_ptr<EuclideanDistribution_<T>> Marginalize(
      beluga_estimation::view<const contiguous_slice> dims) const {
    return std::static_pointer_cast<EuclideanDistribution_<T>>(this->DoMarginalize(dims));
  }

  [[nodiscard]] std::shared_ptr<EuclideanDistribution_<T>> Marginalize(const contiguous_slice& dims) const {
    return Marginalize(ranges::views::single(dims));
  }
};

/// @brief Default-scalar alias for EuclideanDistribution_.
using EuclideanDistribution = EuclideanDistribution_<DefaultScalar>;

}  // namespace beluga_estimation

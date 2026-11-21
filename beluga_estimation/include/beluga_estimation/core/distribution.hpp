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
#include <random>

#include <range/v3/view/single.hpp>

#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>

namespace beluga_estimation {

/**
 * @brief Interface for probability distributions over the manifold.
 * Defines methods to compute mean and covariance, which are essential
 * for inference and optimization.
 */
template <typename T>
class Distribution_ {
 public:
  virtual ~Distribution_() = default;

  /**
   * @brief Returns the dimensionality of the distribution domain space.
   */
  [[nodiscard]] virtual std::size_t DomainSize() const = 0;

  /**
   * @brief Computes the mean of the distribution.
   *
   * @param mean Output span where the computed mean will be stored.
   *             The size of the span must match the dimensionality
   *             of the distribution domain.
   */
  virtual void ComputeMean(std_future::span<T> mean) const = 0;

  /**
   * @brief Computes the covariance matrix of the distribution.
   *
   * @param covariance Output 2D span of size in row-major order.
   *                   The size of the span must typically match
   *                   the dimensionality of the distribution domain
   *                   (though this may vary depending on distribution
   *                   formulation).
   */
  virtual void ComputeCovariance(std_future::mdspan<T, 2> covariance) const = 0;

  /**
   * @brief Computes the log-probability density of a given state.
   *
   * @param value A span containing the state to evaluate.
   * @return The log-probability density evaluated at the given state.
   */
  virtual T LogProbability(std_future::span<const T> value) const = 0;

  /**
   * @brief Generates random samples from this distribution.
   *
   * This provides a concrete, non-templated interface suitable for virtual
   * dispatch and dynamic binding (e.g., Python).
   *
   * @param values A span of spans, where each inner span will store a sampled state.
   * @param gen A random number generator.
   */
  virtual void Sample(view<std_future::span<T>> values, RandomNumberGenerator& gen) const = 0;

  /**
   * @brief Marginalizes this distribution to a subset of dimensions.
   *
   * Each contiguous slice in @p dims selects a set of contiguous dimension
   * indices to retain. The slices must not overlap.
   *
   * @param dims Contiguous slices describing the subset of dimensions to retain.
   * @return A shared pointer to the marginalized distribution, or nullptr
   *   if the marginalization is not supported.
   */
  [[nodiscard]] std::shared_ptr<Distribution_<T>> Marginalize(
      beluga_estimation::view<const contiguous_slice> dims) const {
    return DoMarginalize(dims);
  }

  /**
   * @brief Marginalizes this distribution to a single contiguous block.
   *
   * @param dims A single contiguous slice describing the dimension range to retain.
   * @return A shared pointer to the marginalized distribution, or nullptr.
   */
  [[nodiscard]] std::shared_ptr<Distribution_<T>> Marginalize(const contiguous_slice& dims) const {
    return DoMarginalize(ranges::views::single(dims));
  }

 protected:
  virtual std::shared_ptr<Distribution_<T>> DoMarginalize(beluga_estimation::view<const contiguous_slice> dims) const {
    return nullptr;
  }
};

using Distribution = Distribution_<DefaultScalar>;

}  // namespace beluga_estimation

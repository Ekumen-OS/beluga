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
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <range/v3/view/generate.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/euclidean_distribution.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief Mixture of weighted Dirac deltas over $\mathbb{R}^N$.
 *
 * Represents $p(x) = \sum_{i=1}^K w_i \, \delta(x - c_i)$ where each
 * $c_i \in \mathbb{R}^N$ is a point and $w_i$ are unnormalized weights.
 * Weights are normalized to sum to one on construction. Duplicate point
 * values are not deduplicated — the same coordinate may appear multiple
 * times, in which case their weights are kept separate.
 *
 * Mean and covariance are computed in the Euclidean sense:
 * - Mean:       $\sum_i p_i \cdot c_i$
 * - Covariance: $\sum_i p_i \cdot (c_i - \bar{c})(c_i - \bar{c})^\top$
 *
 * The log-probability is $-\infty$ for all inputs (Dirac densities are
 * zero almost everywhere), making this distribution unsuitable for
 * likelihood-based inference. It is used to represent empirical
 * distributions in resampling and related Monte Carlo steps.
 *
 * @tparam T Scalar type (e.g., double, float).
 * @tparam N Dimension of the Euclidean space each point lives in.
 */
template <typename T, int N = Eigen::Dynamic>
class DiracMixtureDistribution_ : public EuclideanDistribution_<T> {
 public:
  // Bring base class span-based methods into overload set
  // (hidden by our typed overrides of the same name).
  using EuclideanDistribution_<T>::ComputeMean;
  using EuclideanDistribution_<T>::ComputeCovariance;
  using EuclideanDistribution_<T>::LogProbability;
  using EuclideanDistribution_<T>::Sample;

  /**
   * @brief Constructs a Dirac mixture from points and unnormalized weights.
   *
   * Points are stored as-is without deduplication. If the same coordinate
   * appears multiple times, each occurrence retains its own weight.
   *
   * @param points Points $(c_1, \ldots, c_K)$, each in $\mathbb{R}^N$.
   * @param weights Non-negative unnormalized weights $(w_1, \ldots, w_K)$.
   *   Must sum to a positive value.
   */
  DiracMixtureDistribution_(std::vector<Vector<T, N>> points, std::vector<T> weights)
      : points_(std::move(points)), probabilities_(std::move(weights)) {
    BELUGA_ESTIMATION_DEMAND(!points_.empty(), "points must not be empty");
    BELUGA_ESTIMATION_DEMAND(points_.size() == probabilities_.size(), "points and weights must have the same size");
    const T total = std::accumulate(probabilities_.begin(), probabilities_.end(), T{0});
    BELUGA_ESTIMATION_DEMAND(total > T{0}, "sum of weights must be positive");
    for (T& p : probabilities_) {
      p /= total;
    }
    categorical_ = std::discrete_distribution<std::size_t>(probabilities_.begin(), probabilities_.end());
  }

  /**
   * @brief Computes the probability-weighted mean $\sum_i p_i \cdot c_i$.
   */
  void ComputeMean(beluga_estimation::Ref<VectorX<T>> mean) const override {
    mean.setZero();
    for (std::size_t i = 0; i < points_.size(); ++i) {
      mean.noalias() += probabilities_[i] * points_[i];
    }
  }

  /**
   * @brief Computes the probability-weighted covariance
   *   $\sum_i p_i \cdot (c_i - \bar{c})(c_i - \bar{c})^\top$.
   */
  void ComputeCovariance(beluga_estimation::Ref<MatrixX<T>> cov) const override {
    Vector<T, N> mean_vec(DomainSize());
    ComputeMean(mean_vec);

    cov.setZero();
    for (std::size_t i = 0; i < points_.size(); ++i) {
      const auto diff = points_[i] - mean_vec;
      cov.noalias() += probabilities_[i] * diff * diff.transpose();
    }
  }

  /**
   * @brief Returns $-\infty$ for all inputs.
   *
   * Dirac densities are zero almost everywhere, so the log-density is
   * $-\infty$ at every point. This distribution is not suitable for
   * likelihood-based inference.
   */
  T LogProbability(beluga_estimation::Ref<const VectorX<T>> /*value*/) const override {
    return -std::numeric_limits<T>::infinity();
  }

  /**
   * @brief Draws samples by categorical selection among the mixture points.
   *
   * Each output sample is one of the $K$ points $c_i$, drawn with
   * probability $p_i$.
   */
  void Sample(beluga_estimation::view<beluga_estimation::Ref<VectorX<T>>> values, RandomNumberGenerator& gen)
      const override {
    for (auto ref : values) {
      ref.noalias() = points_[categorical_(gen)];
    }
  }

  /**
   * @brief Draws samples (typed convenience overload).
   *
   * @copydetails Sample(beluga_estimation::view<beluga_estimation::Ref<VectorX<T>>>, RandomNumberGenerator&)
   */
  void Sample(beluga_estimation::view<Vector<T, N>&> values, RandomNumberGenerator& rng) const {
    auto ref_view = ranges::views::transform(values, [](auto& vec) { return beluga_estimation::Ref<VectorX<T>>(vec); });
    this->Sample(beluga_estimation::view<beluga_estimation::Ref<VectorX<T>>>{std::move(ref_view)}, rng);
  }

  /**
   * @brief Allocates and returns @p count samples.
   *
   * @copydetails Sample(beluga_estimation::view<beluga_estimation::Ref<VectorX<T>>>, RandomNumberGenerator&)
   */
  std::vector<Vector<T, N>> Sample(std::size_t count, RandomNumberGenerator& gen) const {
    std::vector<Vector<T, N>> samples(count, Vector<T, N>(static_cast<Eigen::Index>(DomainSize())));
    Sample(beluga_estimation::to_view(samples), gen);
    return samples;
  }

  /**
   * @brief Returns a lazy infinite range of sampled points.
   *
   * @copydetails Sample(beluga_estimation::view<beluga_estimation::Ref<VectorX<T>>>, RandomNumberGenerator&)
   */
  auto Sample(RandomNumberGenerator& gen) const {
    return ranges::views::generate([this, &gen]() -> Vector<T, N> { return points_[categorical_(gen)]; });
  }

  /**
   * @brief Returns the dimensionality of this distribution.
   * @return N, the dimension of each point.
   */
  [[nodiscard]] std::size_t DomainSize() const override {
    if constexpr (N == Eigen::Dynamic) {
      return points_.empty() ? 0 : static_cast<std::size_t>(points_[0].size());
    } else {
      return static_cast<std::size_t>(N);
    }
  }

  /// @brief Returns the mixture points $(c_1, \ldots, c_K)$.
  ///
  /// The same coordinate value may appear multiple times (the constructor
  /// does not deduplicate). Callers should treat the point set as a
  /// multiset.
  const std::vector<Vector<T, N>>& Points() const { return points_; }

  /// @brief Returns the normalized probabilities $(p_1, \ldots, p_K)$.
  ///
  /// Each $p_i$ is the normalized form of the $i$-th weight passed to the
  /// constructor, so $\sum_i p_i = 1$.
  const std::vector<T>& Probabilities() const { return probabilities_; }

  /**
   * @brief Marginalizes this distribution to a subset of dimensions.
   *
   * Marginalization is **approximate**: it drops unselected dimensions from
   * each Dirac point, producing a Dirac mixture in the lower-dimensional
   * space. The result is only as accurate as the original Dirac mixture
   * representation of the true marginal distribution.
   *
   * @param dims A single slice describing the dimension range to retain.
   * @return A shared pointer to the marginalized distribution, or nullptr
   *   if @p dims is empty or contains overlapping intervals.
   */
  [[nodiscard]] std::shared_ptr<DiracMixtureDistribution_<T>> Marginalize(const contiguous_slice& dims) const {
    return Marginalize(ranges::views::single(dims));
  }

  /// @overload
  [[nodiscard]] std::shared_ptr<DiracMixtureDistribution_<T>> Marginalize(
      beluga_estimation::view<const contiguous_slice> dims) const {
    return std::static_pointer_cast<DiracMixtureDistribution_<T>>(DoMarginalize(dims));
  }

 protected:
  /// @copydoc Distribution_::DoMarginalize
  ///
  /// @note Marginalization is approximate. @see Marginalize for details.
  [[nodiscard]] std::shared_ptr<Distribution_<T>> DoMarginalize(
      beluga_estimation::view<const contiguous_slice> dims) const override {
    // -- Step 1: Validate intervals --
    if (!disjoint(dims)) {
      return nullptr;
    }

    // -- Step 2: Compute output dimension --
    std::size_t marginal_ndim = 0;
    for (const auto& dim : dims) {
      const std::size_t start = dim.start.value_or(0);
      const std::size_t end = dim.end.value_or(DomainSize());
      marginal_ndim += end - start;
    }
    if (marginal_ndim == 0) {
      return nullptr;
    }

    // -- Step 3: Project each point to the retained dimensions --
    //
    // Marginalization extracts the selected dimensions from each Dirac
    // point, forming a new Dirac mixture in the lower-dimensional space.
    // Weights are preserved unchanged.

    std::vector<beluga_estimation::VectorX<T>> marginal_points(
        points_.size(), beluga_estimation::VectorX<T>(marginal_ndim));
    for (std::size_t i = 0; i < points_.size(); ++i) {
      Eigen::Index offset = 0;
      for (const auto& dim : dims) {
        const Eigen::Index local_offset = static_cast<Eigen::Index>(dim.start.value_or(0));
        const Eigen::Index size = static_cast<Eigen::Index>(dim.end.value_or(DomainSize())) - local_offset;
        marginal_points[i].segment(offset, size) = points_[i].segment(local_offset, size);
        offset += size;
      }
    }
    return std::make_shared<DiracMixtureDistribution_<T>>(std::move(marginal_points), probabilities_);
  }

 private:
  std::vector<beluga_estimation::Vector<T, N>> points_;  ///< Points $(c_1, \ldots, c_K)$.
  std::vector<T> probabilities_;                         ///< Normalized probability vector $(p_1, \ldots, p_K)$.

  /// Pre-built categorical distribution. Mutable because `operator()` advances
  /// internal RNG state without changing the distribution parameters.
  mutable std::discrete_distribution<std::size_t> categorical_;
};

using DiracMixtureDistribution = DiracMixtureDistribution_<DefaultScalar>;

template <int N>
using DiracMixtureNDistribution = DiracMixtureDistribution_<DefaultScalar, N>;

}  // namespace beluga_estimation

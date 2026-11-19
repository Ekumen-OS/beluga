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
#include <optional>
#include <random>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>
#include <range/v3/view/single.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/probability.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/euclidean_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief Multivariate normal (Gaussian) distribution over $\mathbb{R}^N$.
 *
 * Represents $\mathcal{N}(\mu, \Sigma)$ where $\mu \in \mathbb{R}^N$ is the
 * mean and $\Sigma \in \mathbb{R}^{N \times N}$ is the covariance matrix.
 *
 * The covariance matrix must be symmetric and positive definite.
 * Sampling uses a cached Cholesky decomposition computed on first use.
 *
 * @tparam T Scalar type (e.g., double, float).
 * @tparam N The dimensionality of the distribution (or @p Eigen::Dynamic).
 */
template <typename T, int N = Eigen::Dynamic>
class NormalDistribution_ : public EuclideanDistribution_<T> {
 public:
  // Bring base class span-based convenience methods into overload set
  // (they are hidden by our typed overrides of the same name).
  using EuclideanDistribution_<T>::ComputeMean;
  using EuclideanDistribution_<T>::ComputeCovariance;
  using EuclideanDistribution_<T>::LogProbability;
  using EuclideanDistribution_<T>::Sample;

  /**
   * @brief Constructs a multivariate normal distribution.
   *
   * @param mean The mean vector of size $N$.
   * @param covariance The $N \times N$ covariance matrix. Must be symmetric and positive definite.
   */
  NormalDistribution_(const Vector<T, N>& mean, const Matrix<T, N, N>& covariance)
      : mean_(mean), covariance_(covariance) {
    BELUGA_ESTIMATION_ASSERT(covariance.rows() == covariance.cols(), "covariance matrix must be square");
    BELUGA_ESTIMATION_ASSERT(mean.size() == covariance.rows(), "mean size must match covariance dimensions");
    BELUGA_ESTIMATION_ASSERT(
        IsCovarianceMatrix(covariance), "covariance matrix must be symmetric and positive definite");
  }

  /**
   * @brief Constructs a zero mean multivariate normal distribution.
   *
   * @param covariance The $N \times N$ covariance matrix. Must be symmetric and positive definite.
   */
  explicit NormalDistribution_(const Matrix<T, N, N>& covariance)
      : NormalDistribution_(Vector<T, N>::Zero(covariance.rows()), covariance) {}

  /**
   * @brief Retrieves the mean vector.
   * @return A const reference to the mean vector of size $N$.
   */
  const Vector<T, N>& Mean() const { return mean_; }

  /**
   * @brief Retrieves the covariance matrix.
   * @return A const reference to the $N \times N$ covariance matrix.
   */
  const Matrix<T, N, N>& Covariance() const { return covariance_; }

  /**
   * @brief Computes the mean of the distribution (typed interface).
   *
   * Eigen handles assignment from our internal fixed/dynamic mean
   * vector into the caller's dynamic-sized @c Ref.
   */
  void ComputeMean(beluga_estimation::Ref<VectorX<T>> mean) const override { mean.noalias() = mean_; }

  /**
   * @brief Computes the covariance matrix (typed interface).
   */
  void ComputeCovariance(beluga_estimation::Ref<MatrixX<T>> cov) const override { cov.noalias() = covariance_; }

  /**
   * @brief Computes the log-probability density at a given state (typed interface).
   */
  T LogProbability(beluga_estimation::Ref<const VectorX<T>> value) const override {
    ComputeCholeskyIfNecessary();
    const Vector<T, N> error = value - mean_;
    const Vector<T, N> l_inv_error = covariance_llt_->matrixL().solve(error);
    const T mahalanobis_sq = l_inv_error.squaredNorm();
    static const T kLog2Pi = static_cast<T>(std::log(2 * M_PI));
    return static_cast<T>(-0.5) * (mean_.size() * kLog2Pi + *log_det_sigma_ + mahalanobis_sq);
  }

  /**
   * @brief Generates random samples from this distribution (typed interface).
   */
  void Sample(beluga_estimation::view<beluga_estimation::Ref<VectorX<T>>> values, RandomNumberGenerator& gen)
      const override {
    ComputeCholeskyIfNecessary();
    Vector<T, N> z(mean_.size());
    std::normal_distribution<T> standard_normal;
    for (auto ref : values) {
      for (Eigen::Index i = 0; i < z.size(); ++i) {
        z[i] = standard_normal(gen);
      }
      ref.noalias() = mean_ + covariance_llt_->matrixL() * z;
    }
  }

  /**
   * @brief Generates random samples (convenience overload).
   *
   * Allocates and returns a vector of sampled $N$-dimensional vectors.
   */
  std::vector<Vector<T, N>> Sample(std::size_t count, RandomNumberGenerator& rng) const {
    std::vector<Vector<T, N>> samples(count, Vector<T, N>(mean_.size()));
    Sample(beluga_estimation::to_view(samples), rng);
    return samples;
  }

  /**
   * @brief Returns the dimensionality of this distribution.
   * @return N, or the runtime dimension when N is Dynamic.
   */
  [[nodiscard]] std::size_t DomainSize() const override {
    if constexpr (N != Eigen::Dynamic) {
      return static_cast<std::size_t>(N);
    } else {
      return static_cast<std::size_t>(mean_.size());
    }
  }

  /**
   * @brief Marginalizes this distribution to a subset of dimensions.
   *
   * Returns a normal distribution over the retained dimensions only.
   * Since normal marginalization simply drops the non-retained dimensions,
   * the result's mean is the sub-vector and its covariance is the sub-matrix
   * extracted at the retained indices.
   *
   * @param dims A single slice describing the dimension range to retain.
   * @return A shared pointer to the marginalized distribution, or nullptr
   *   if @p dims is empty or contains overlapping intervals.
   */
  [[nodiscard]] std::shared_ptr<NormalDistribution_<T>> Marginalize(const contiguous_slice& dims) const {
    return Marginalize(ranges::views::single(dims));
  }

  /// @overload
  [[nodiscard]] std::shared_ptr<NormalDistribution_<T>> Marginalize(
      beluga_estimation::view<const contiguous_slice> dims) const {
    return std::static_pointer_cast<NormalDistribution_<T>>(DoMarginalize(dims));
  }

 protected:
  /// @copydoc Distribution_::DoMarginalize
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

    // -- Step 3: Extract marginal mean and covariance blocks --
    //
    // Normal marginalization drops the non-retained dimensions. The marginal
    // mean and covariance are simply the sub-vector and sub-matrix extracted
    // at the retained indices. For each retained interval:
    //
    //   1. Copy the corresponding segment of the mean vector.
    //   2. Copy the diagonal covariance block (interval × interval).
    //   3. Copy the off-diagonal covariance blocks with each subsequent
    //      retained interval (symmetric, copied in both orders).

    beluga_estimation::VectorX<T> marginal_mean(marginal_ndim);
    beluga_estimation::MatrixX<T> marginal_covariance(marginal_ndim, marginal_ndim);

    Eigen::Index offset = 0;
    for (auto it = dims.begin(); it != dims.end(); ++it) {
      const auto& dim = *it;
      const Eigen::Index start = static_cast<Eigen::Index>(dim.start.value_or(0));
      const Eigen::Index size = static_cast<Eigen::Index>(dim.end.value_or(DomainSize())) - start;
      // (a) Copy mean segment for this interval.
      marginal_mean.segment(offset, size) = mean_.segment(start, size);

      // (b) Copy diagonal covariance block: retained × retained.
      marginal_covariance.block(offset, offset, size, size) = covariance_.block(start, start, size, size);

      // (c) Copy off-diagonal blocks with all subsequent intervals.
      Eigen::Index other_offset = offset + size;
      for (auto jt = std::next(it); jt != dims.end(); ++jt) {
        const auto& other_dim = *jt;
        const Eigen::Index other_start = static_cast<Eigen::Index>(other_dim.start.value_or(0));
        const Eigen::Index other_size = static_cast<Eigen::Index>(other_dim.end.value_or(DomainSize())) - other_start;

        // Block (current, other)
        marginal_covariance.block(offset, other_offset, size, other_size) =
            covariance_.block(start, other_start, size, other_size);

        // Block (other, current) — symmetric
        marginal_covariance.block(other_offset, offset, other_size, size) =
            covariance_.block(other_start, start, other_size, size);

        other_offset += other_size;
      }
      offset += size;
    }
    return std::make_shared<NormalDistribution_<T>>(std::move(marginal_mean), std::move(marginal_covariance));
  }

 private:
  /// @brief Computes the Cholesky decomposition on first access.
  ///
  /// Both `LogProbability` and `Sample` require $L$ where $\Sigma = LL^\top$.
  /// The decomposition and the log-determinant are computed on first use and
  /// cached for subsequent calls.
  void ComputeCholeskyIfNecessary() const {
    if (!covariance_llt_.has_value()) {
      covariance_llt_ = covariance_.llt();
      log_det_sigma_ = 2 * covariance_llt_->matrixLLT().diagonal().array().log().sum();
    }
  }

  Vector<T, N> mean_;
  Matrix<T, N, N> covariance_;

  mutable std::optional<Eigen::LLT<Matrix<T, N, N>>> covariance_llt_;
  mutable std::optional<T> log_det_sigma_;
};

using NormalDistribution = NormalDistribution_<DefaultScalar>;

template <int N>
using NormalNDistribution = NormalDistribution_<DefaultScalar, N>;

}  // namespace beluga_estimation

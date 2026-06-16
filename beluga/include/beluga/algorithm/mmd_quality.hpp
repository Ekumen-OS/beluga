// Copyright 2024 Ekumen, Inc.
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

#ifndef BELUGA_ALGORITHM_MMD_QUALITY_HPP
#define BELUGA_ALGORITHM_MMD_QUALITY_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <sophus/se2.hpp>

#include <range/v3/range/conversion.hpp>
#include <range/v3/utility/random.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga/random/multivariate_distribution_traits.hpp>
#include <beluga/random/multivariate_normal_distribution.hpp>
#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views/particles.hpp>
#include <beluga/views/sample.hpp>

/**
 * \file
 * \brief Implementation of an MMD-based quality metric for particle filter estimates.
 *
 * Computes a p-value via Gretton's linear-time paired MMD test
 * (Gretton et al., JMLR 2012, Lemma 6 / Lemma 14, Theorem 10)
 * on SE(2) using a product kernel:
 *   - Anisotropic Gaussian RBF for (x, y)
 *   - Von Mises (cos⁻¹) kernel for yaw
 */

namespace beluga {

/// Configuration for the MMD product kernel.
/**
 * The kernel is k(z, z') = k_xy((x,y), (x',y')) * k_yaw(theta, theta')
 * where:
 *   - k_xy uses an anisotropic squared-exponential (Gaussian RBF)
 *   - k_yaw uses a von Mises kernel: exp(kappa * (cos(dtheta) - 1))
 *     with kappa = 1 / sigma_yaw²
 *
 * The product is bounded in [0, 1] with k(z, z) = 1, satisfying
 * the K=1 condition for Theorem 10.
 */
struct MmdKernelConfig {
  /// Gaussian kernel bandwidth for x [m].
  double sigma_kx = 0.1;

  /// Gaussian kernel bandwidth for y [m].
  double sigma_ky = 0.1;

  /// Standard deviation for yaw [rad], used to compute kappa = 1 / sigma_yaw².
  double sigma_yaw = 0.05;

  /// Compute von Mises concentration from the configured yaw stddev.
  [[nodiscard]] double kappa() const noexcept { return 1.0 / (sigma_yaw * sigma_yaw); }
};

namespace detail {

/// \cond detail

/// Product kernel functor: anisotropic Gaussian (x,y) × von Mises (yaw).
struct mmd_product_kernel_fn {
  /// Evaluate the kernel on two state vectors [x, y, yaw].
  double operator()(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const MmdKernelConfig& config) const noexcept {
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    const double dtheta = a.z() - b.z();

    // Anisotropic Gaussian RBF for (x, y)
    const double k_xy = std::exp(
        -0.5 * (dx * dx / (config.sigma_kx * config.sigma_kx) + dy * dy / (config.sigma_ky * config.sigma_ky)));

    // Von Mises kernel for yaw: exp(kappa * (cos(dtheta) - 1))
    const double k_yaw = std::exp(config.kappa() * (std::cos(dtheta) - 1.0));

    return k_xy * k_yaw;
  }
};

/// Compute the linear-time paired MMD² estimate (Gretton Lemma 6 / Lemma 14).
/**
 * Given paired samples X = {x_1, ..., x_n} and Y = {y_1, ..., y_n},
 * the linear-time statistic averages the h-statistic over consecutive pairs:
 *
 *   MMD²_l = (2/n) * Σ_{i=1}^{n/2} h(z_{2i-1}, z_{2i})
 *
 * where h(z_i, z_j) = k(x_i, x_j) + k(y_i, y_j) - k(x_i, y_j) - k(y_i, x_j).
 *
 * Returns 0.0 if n < 2.
 */
inline double compute_paired_mmd2(
    const std::vector<Eigen::Vector3d>& X,
    const std::vector<Eigen::Vector3d>& Y,
    const MmdKernelConfig& config) {
  assert(X.size() == Y.size());
  const auto n = X.size();
  if (n < 2) {
    return 0.0;
  }

  const auto num_pairs = n / 2;
  double sum = 0.0;

  const mmd_product_kernel_fn kernel;

  for (std::size_t i = 0; i < num_pairs; ++i) {
    const auto& x1 = X[2 * i];
    const auto& x2 = X[2 * i + 1];
    const auto& y1 = Y[2 * i];
    const auto& y2 = Y[2 * i + 1];

    // h(z_{2i-1}, z_{2i}) = k(x1,x2) + k(y1,y2) - k(x1,y2) - k(y1,x2)
    sum += kernel(x1, x2, config) + kernel(y1, y2, config) - kernel(x1, y2, config) - kernel(y1, x2, config);
  }

  // MMD²_l = (2/n) * Σ h
  return 2.0 * sum / static_cast<double>(n);
}

/// Compute the n-samples p-value for linear-time paired MMD² on two vector sets.
/**
 * \tparam URNG A UniformRandomBitGenerator (e.g. std::mt19937).
 * \param X First set of samples (as [x, y, yaw] vectors).
 * \param Y Second set of samples (as [x, y, yaw] vectors), same size as X.
 * \param sample_count Number of samples in each set.
 * \param config Kernel configuration.
 * \return p-value in [0, 1].
 */
inline double mmd_pvalue_from_vectors(
    const std::vector<Eigen::Vector3d>& X,
    const std::vector<Eigen::Vector3d>& Y,
    std::size_t sample_count,
    const MmdKernelConfig& config) {
  const double mmd2 = compute_paired_mmd2(X, Y, config);
  return std::min(1.0, std::exp(-static_cast<double>(sample_count) * mmd2 / 16.0));
}

/// Implementation of the MMD quality niebloid.
struct mmd_quality_fn {
  // -- Parametric mode (posterior Gaussian) ---------------------------------

  /// Evaluate quality from Gaussian posterior and Gaussian reference.
  /**
   * \tparam URNG A UniformRandomBitGenerator (defaults to the ranges default engine).
   * \param post_mean Posterior distribution mean.
   * \param post_cov Posterior distribution covariance.
   * \param ref_mean Reference distribution mean.
   * \param ref_cov Reference distribution covariance.
   * \param sample_count Number of samples from each distribution.
   * \param config Kernel configuration.
   * \param gen Random generator instance.
   * \return p-value in [0, 1].
   */
  template <class URNG = typename ranges::detail::default_random_engine>
  double operator()(
      const Sophus::SE2d& post_mean,
      const Sophus::Matrix3d& post_cov,
      const Sophus::SE2d& ref_mean,
      const Sophus::Matrix3d& ref_cov,
      std::size_t sample_count,
      const MmdKernelConfig& config,
      URNG& gen = ranges::detail::get_random_engine()) const {
    using Distribution = MultivariateNormalDistribution<Sophus::SE2d>;

    if (sample_count < 2) {
      sample_count = 2;
    }

    using Traits = multivariate_distribution_traits<Sophus::SE2d>;
    const auto to_vector = [](const auto& s) { return Traits::to_vector(s); };

    const auto X = beluga::views::sample(Distribution{post_mean, post_cov}, gen) | ranges::views::transform(to_vector) |
                   ranges::views::take_exactly(sample_count) | ranges::to<std::vector>;

    const auto Y = beluga::views::sample(Distribution{ref_mean, ref_cov}, gen) | ranges::views::transform(to_vector) |
                   ranges::views::take_exactly(sample_count) | ranges::to<std::vector>;

    return mmd_pvalue_from_vectors(X, Y, sample_count, config);
  }

  /// Evaluate quality with default sample count (500).
  template <class URNG = typename ranges::detail::default_random_engine>
  double operator()(
      const Sophus::SE2d& post_mean,
      const Sophus::Matrix3d& post_cov,
      const Sophus::SE2d& ref_mean,
      const Sophus::Matrix3d& ref_cov,
      const MmdKernelConfig& config,
      URNG& gen = ranges::detail::get_random_engine()) const {
    constexpr auto kDefaultSampleCount = 500;
    return (*this)(post_mean, post_cov, ref_mean, ref_cov, kDefaultSampleCount, config, gen);
  }

  // -- Particle mode (weighted particles) -----------------------------------

  /// Evaluate quality from weighted particles and a Gaussian reference.
  /**
   * Resamples `sample_count` particles (with replacement) using weighted
   * resampling via beluga::views::sample, then compares to `sample_count`
   * draws from the reference Gaussian.
   *
   * \tparam ParticleRange A range satisfying is_particle_range.
   * \tparam URNG A UniformRandomBitGenerator (defaults to the ranges default engine).
   * \param particles Weighted particle range.
   * \param ref_mean Reference distribution mean.
   * \param ref_cov Reference distribution covariance.
   * \param sample_count Number of samples from each distribution.
   * \param config Kernel configuration.
   * \param gen Random generator instance.
   * \return p-value in [0, 1].
   */
  template <class ParticleRange, class URNG = typename ranges::detail::default_random_engine>
  double operator()(
      ParticleRange&& particles,
      const Sophus::SE2d& ref_mean,
      const Sophus::Matrix3d& ref_cov,
      std::size_t sample_count,
      const MmdKernelConfig& config,
      URNG& gen = ranges::detail::get_random_engine()) const {
    static_assert(beluga::is_particle_range_v<ParticleRange>, "Input must satisfy is_particle_range");

    if (ranges::size(particles) == 0) {
      return 1.0;
    }

    if (sample_count < 2) {
      sample_count = 2;
    }

    using Distribution = MultivariateNormalDistribution<Sophus::SE2d>;
    using Traits = multivariate_distribution_traits<Sophus::SE2d>;

    auto to_vector = [](const auto& s) { return Traits::to_vector(s); };
    auto extract_state = [](const auto& p) { return beluga::state(p); };

    // Weighted resample from the particle set via beluga::views::sample.
    const auto X = beluga::views::sample(particles, gen) | ranges::views::transform(extract_state) |
                   ranges::views::transform(to_vector) | ranges::views::take_exactly(sample_count) |
                   ranges::to<std::vector>;

    // Sample from reference
    const auto Y = beluga::views::sample(Distribution{ref_mean, ref_cov}, gen) | ranges::views::transform(to_vector) |
                   ranges::views::take_exactly(sample_count) | ranges::to<std::vector>;

    return mmd_pvalue_from_vectors(X, Y, sample_count, config);
  }

  /// Evaluate quality using the particle count as sample count.
  template <class ParticleRange, class URNG = typename ranges::detail::default_random_engine>
  double operator()(
      ParticleRange&& particles,
      const Sophus::SE2d& ref_mean,
      const Sophus::Matrix3d& ref_cov,
      const MmdKernelConfig& config,
      URNG& gen = ranges::detail::get_random_engine()) const {
    const auto n = ranges::size(particles);
    return (*this)(std::forward<ParticleRange>(particles), ref_mean, ref_cov, n, config, gen);
  }
};

/// \endcond

}  // namespace detail

/// Compute the MMD-based quality p-value for a particle filter estimate.
/**
 * This niebloid provides the primary entry point for computing the quality
 * of a particle filter estimate using the maximum mean discrepancy (MMD)
 * two-sample test.
 *
 * Two modes are available:
 *   - **Parametric mode**: given the posterior mean and covariance (e.g. from
 *     cluster_based_estimate), sample from a Gaussian posterior and reference.
 *   - **Particle mode**: given a weighted particle range, resample with
 *     replacement and compare to a Gaussian reference.
 *
 * When `sample_count` is omitted:
 *   - Parametric mode defaults to 500 samples.
 *   - Particle mode uses the number of particles as the sample count.
 *
 * Each overload accepts an optional URNG in the last position (defaults to
 * the ranges thread-local default engine).
 *
 * The returned p-value represents the probability that the posterior and
 * reference distributions are the same under H0. A high p-value (close to 1)
 * indicates the filter is behaving as expected; a low p-value (close to 0)
 * suggests divergence.
 */
inline constexpr detail::mmd_quality_fn mmd_quality;

}  // namespace beluga

#endif  // BELUGA_ALGORITHM_MMD_QUALITY_HPP

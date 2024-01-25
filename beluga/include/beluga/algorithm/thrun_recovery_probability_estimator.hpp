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

#ifndef BELUGA_ALGORITHM_THRUN_RECOVERY_PROBABILITY_ESTIMATOR_HPP
#define BELUGA_ALGORITHM_THRUN_RECOVERY_PROBABILITY_ESTIMATOR_HPP

#include <beluga/algorithm/exponential_filter.hpp>
#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views/particles.hpp>

#include <range/v3/numeric/accumulate.hpp>

namespace beluga {

/// Random particle probability estimator.
/**
 * This class implements an estimator for what probability to use for injecting random
 * particles (not sampled directly from the particle set) during the resampling step of a
 * particle filter. The inclusion of random samples enhances the filter's ability to recover
 * in case it converges to an incorrect estimate, thereby adding an extra layer of robustness.
 *
 * This estimator averages the total weight of the particles and computes the ratio
 * between a short-term and a long-term average over time.
 *
 * See Probabilistic Robotics \cite thrun2005probabilistic, Chapter 8.3.3.
 */
class ThrunRecoveryProbabilityEstimator {
 public:
  /// Constructor.
  /**
   * \param alpha_slow Decay rate for the long-term average.
   * \param alpha_fast Decay rate for the short-term average.
   */
  constexpr ThrunRecoveryProbabilityEstimator(double alpha_slow, double alpha_fast) noexcept
      : slow_filter_{alpha_slow}, fast_filter_{alpha_fast} {
    assert(0 < alpha_slow);
    assert(alpha_slow < alpha_fast);
  }

  /// Reset the internal state of the estimator.
  /**
   * It is recommended to reset the estimator after injecting random particles
   * to avoid spiraling off into complete randomness.
   */
  constexpr void reset() noexcept {
    slow_filter_.reset();
    fast_filter_.reset();
  }

  /// Update the estimation based on a particle range.
  /**
   * \param range A range containing particles.
   * \return The estimated random state probability to be used by the particle filter.
   */
  template <class Range>
  constexpr double operator()(Range&& range) {
    static_assert(ranges::sized_range<Range>);
    static_assert(beluga::is_particle_range_v<Range>);
    const std::size_t size = range.size();

    if (size == 0) {
      reset();
      return 0.0;
    }

    const double total_weight = ranges::accumulate(beluga::views::weights(range), 0.0);
    const double average_weight = total_weight / static_cast<double>(size);
    const double fast_average = fast_filter_(average_weight);
    const double slow_average = slow_filter_(average_weight);

    if (std::abs(slow_average) < std::numeric_limits<double>::epsilon()) {
      return 0.0;
    }

    return std::clamp(1.0 - fast_average / slow_average, 0.0, 1.0);
  }

 private:
  ExponentialFilter slow_filter_;  ///< Exponential filter for the long-term average.
  ExponentialFilter fast_filter_;  ///< Exponential filter for the short-term average.
};

}  // namespace beluga

#endif

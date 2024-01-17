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

#ifndef BELUGA_ALGORITHM_EFFECTIVE_SAMPLE_SIZE_HPP
#define BELUGA_ALGORITHM_EFFECTIVE_SAMPLE_SIZE_HPP

#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views/particles.hpp>

#include <range/v3/numeric/accumulate.hpp>

/**
 * \file
 * \brief Implementation of an algorithm to calculate the effective sample size (ESS).
 */

namespace beluga {

/// Calculate the ESS of a given a range of weights.
/**
 * The effective sample size (ESS) is a figure of merit for importance sampling methods' output. It characterizes
 * how well the target posterior distribution is approximated (as it is proportional to the efficiency of distribution
 * parameter estimators, \cite kong1994sequentialimputations, section 4.1). It can be interpreted as the number of
 * samples effectively approximating the distribution, and thus comparing with the total number of samples makes for a
 * good mechanism to detect and react to performance degradation e.g. triggering a resample when the ESS falls below a
 * fraction of the total number of samples.
 *
 * The algorithm is based on \cite grisetti2007selectiveresampling, according to the description given in
 * \cite tiacheng2015resamplingmethods.
 *
 * \tparam Range A [forward range](https://en.cppreference.com/w/cpp/ranges/forward_range).
 * \param range The range of weights.
 */
template <class Range, std::enable_if_t<!is_particle_range_v<Range>, int> = 0>
auto effective_sample_size(Range&& range) {
  const auto total_weight = ranges::accumulate(range, 0.0);

  if (total_weight == 0.0) {
    return 0.0;
  }

  auto normalize_and_square = [total_weight](auto weight) {
    const auto normalized = weight / total_weight;
    return normalized * normalized;
  };

  return 1.0 / ranges::accumulate(range, 0.0, std::plus<>{}, std::move(normalize_and_square));
}

/// Overload for particle ranges.
template <class Range, std::enable_if_t<is_particle_range_v<Range>, int> = 0>
auto effective_sample_size(Range&& range) {
  return effective_sample_size(range | beluga::views::weights);
}

}  // namespace beluga

#endif

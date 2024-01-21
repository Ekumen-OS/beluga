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

#ifndef BELUGA_POLICIES_ON_EFFECTIVE_SIZE_DROP_HPP
#define BELUGA_POLICIES_ON_EFFECTIVE_SIZE_DROP_HPP

#include <beluga/policies/policy.hpp>

/**
 * \file
 * \brief Defines a policy for triggering an action based on the Effective Sample Size (ESS) metric.
 */

namespace beluga::policies {

namespace detail {

/// Implementation detail for a on_effective_size_drop_fn object.
struct on_effective_size_drop_fn {
  /// Overload that implements the condition.
  /**
   * \tparam Range The type of the range containing particles.
   * \param range The range containing particles for which the ESS is calculated.
   * \return True if resampling should be triggered, false otherwise.
   */
  template <class Range>
  constexpr bool operator()(Range&& range) const {
    static_assert(ranges::sized_range<Range>);
    const auto size = static_cast<double>(ranges::size(range));
    return beluga::effective_sample_size(std::forward<Range>(range)) < size / 2.0;
  }
};

}  // namespace detail

/// Policy that can be used to trigger an action based on the Effective Sample Size (ESS) metric.
/**
 * This policy is designed for scenarios where an action is desired when the Effective Sample Size
 * drops below a certain threshold, specifically half the number of particles.
 */
inline constexpr policy_closure<detail::on_effective_size_drop_fn> on_effective_size_drop;

}  // namespace beluga::policies

#endif

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

#ifndef BELUGA_ACTIONS_NORMALIZE_HPP
#define BELUGA_ACTIONS_NORMALIZE_HPP

#include <algorithm>
#include <execution>

#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views/particles.hpp>

#include <range/v3/action/action.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/common.hpp>

namespace beluga::actions {

namespace detail {

/// Implementation detail for a normalize range adaptor object.
struct normalize_base_fn {
  /// Overload that implements the normalize algorithm.
  /**
   * \tparam ExecutionPolicy An [execution policy](https://en.cppreference.com/w/cpp/algorithm/execution_policy_tag_t).
   * \tparam Range An [input range](https://en.cppreference.com/w/cpp/ranges/input_range).
   * \param policy The execution policy to use.
   * \param range An existing range to apply this action to.
   * \param factor The normalization factor.
   */
  template <
      class ExecutionPolicy,
      class Range,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0,
      std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, Range& range, double factor) const -> Range& {
    if (std::abs(factor - 1.0) < std::numeric_limits<double>::epsilon()) {
      return range;  // No change.
    }

    auto weights = [&range]() {
      if constexpr (beluga::is_particle_range_v<Range>) {
        return range | beluga::views::weights | ranges::views::common;
      } else {
        return range | ranges::views::common;
      }
    }();

    std::transform(
        policy,               //
        std::begin(weights),  //
        std::end(weights),    //
        std::begin(weights),  //
        [factor](const auto w) { return w / factor; });
    return range;
  }

  /// Overload that uses a default normalization factor.
  /**
   * The default normalization factor is the total sum of weights.
   */
  template <
      class ExecutionPolicy,
      class Range,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0,
      std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, Range& range) const -> Range& {
    auto weights = [&range]() {
      if constexpr (beluga::is_particle_range_v<Range>) {
        return range | beluga::views::weights | ranges::views::common;
      } else {
        return range | ranges::views::common;
      }
    }();

    const double total_weight = ranges::accumulate(weights, 0.0);
    return (*this)(std::forward<ExecutionPolicy>(policy), range, total_weight);
  }

  /// Overload that re-orders arguments from an action closure.
  template <
      class Range,
      class ExecutionPolicy,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>, int> = 0>
  constexpr auto operator()(Range&& range, double factor, ExecutionPolicy policy) const -> Range& {
    return (*this)(std::move(policy), std::forward<Range>(range), factor);
  }

  /// Overload that re-orders arguments from an action closure.
  template <
      class Range,
      class ExecutionPolicy,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>, int> = 0>
  constexpr auto operator()(Range&& range, ExecutionPolicy policy) const -> Range& {
    return (*this)(std::move(policy), std::forward<Range>(range));
  }

  /// Overload that returns an action closure to compose with other actions.
  template <class ExecutionPolicy, std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>, int> = 0>
  constexpr auto operator()(ExecutionPolicy policy, double factor) const {
    return ranges::make_action_closure(ranges::bind_back(normalize_base_fn{}, factor, std::move(policy)));
  }

  /// Overload that returns an action closure to compose with other actions.
  template <class ExecutionPolicy, std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>, int> = 0>
  constexpr auto operator()(ExecutionPolicy policy) const {
    return ranges::make_action_closure(ranges::bind_back(normalize_base_fn{}, std::move(policy)));
  }
};

/// Implementation detail for a normalize range adaptor object with a default execution policy.
struct normalize_fn : public normalize_base_fn {
  using normalize_base_fn::operator();

  /// Overload that defines a default execution policy.
  template <class Range, std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(Range&& range, double factor) const -> Range& {
    return (*this)(std::execution::seq, std::forward<Range>(range), factor);
  }

  /// Overload that defines a default execution policy.
  template <class Range, std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(Range&& range) const -> Range& {
    return (*this)(std::execution::seq, std::forward<Range>(range));
  }

  /// Overload that returns an action closure to compose with other actions.
  constexpr auto operator()(double factor) const {
    return ranges::make_action_closure(ranges::bind_back(normalize_fn{}, factor));
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// can normalize a range of values (or a range of particles).
/**
 * The `normalize` range adaptor allows users to normalize the weights of a range
 * (or a range of particles) by dividing each weight by a specified normalization factor.
 *
 * If none is specified, the default normalization factor corresponds to the total sum of weights
 * in the given range.
 */
inline constexpr ranges::actions::action_closure<detail::normalize_fn> normalize;

}  // namespace beluga::actions

#endif

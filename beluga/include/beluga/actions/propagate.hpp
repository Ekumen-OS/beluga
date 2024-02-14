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

#ifndef BELUGA_ACTIONS_PROPAGATE_HPP
#define BELUGA_ACTIONS_PROPAGATE_HPP

#include <algorithm>
#include <execution>
#include <type_traits>

#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views/particles.hpp>

#include <range/v3/action/action.hpp>
#include <range/v3/utility/random.hpp>
#include <range/v3/view/common.hpp>

namespace beluga::actions {

namespace detail {

/// Implementation detail for a propagate range adaptor object.
struct propagate_base_fn {
  /// Overload that implements the propagate algorithm.
  /**
   * \tparam ExecutionPolicy An [execution policy](https://en.cppreference.com/w/cpp/algorithm/execution_policy_tag_t).
   * \tparam Range An [input range](https://en.cppreference.com/w/cpp/ranges/input_range) of particles.
   * \tparam StateSamplingFunction A callable that samples a posterior state given a prior state. Callables satisfying
   * \ref StateSamplingFunctionPage are also supported. This will be bound to the default random number generator for
   * ranges.
   * \param policy The execution policy to use.
   * \param range An existing range of particles to apply this action to.
   * \param fn A state sampling function instance.
   */
  template <
      class ExecutionPolicy,
      class Range,
      class StateSamplingFunction,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0,
      std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, Range& range, StateSamplingFunction fn) const -> Range& {
    static_assert(beluga::is_particle_range_v<Range>);
    auto states = range | beluga::views::states | ranges::views::common;

    auto unary_fn = [&]() {
      using States = decltype(states);
      using State = ranges::range_value_t<States>;
      using Generator = decltype(ranges::detail::get_random_engine());
      if constexpr (std::is_invocable_v<StateSamplingFunction, State, Generator>) {
        return [fn = std::move(fn)](const State& state) { return fn(state, ranges::detail::get_random_engine()); };
      } else {
        return std::move(fn);
      }
    }();

    std::transform(
        policy,              // rvalue policies are not supported in some STL implementations
        std::begin(states),  //
        std::end(states),    //
        std::begin(states),  //
        std::move(unary_fn));
    return range;
  }

  /// Overload that re-orders arguments from a view closure.
  template <
      class Range,
      class StateSamplingFunction,
      class ExecutionPolicy,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>, int> = 0>
  constexpr auto operator()(Range&& range, StateSamplingFunction fn, ExecutionPolicy policy) const -> Range& {
    return (*this)(std::move(policy), std::forward<Range>(range), std::move(fn));
  }

  /// Overload that returns a view closure to compose with other views.
  template <
      class ExecutionPolicy,        //
      class StateSamplingFunction,  //
      std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>, int> = 0>
  constexpr auto operator()(ExecutionPolicy policy, StateSamplingFunction fn) const {
    return ranges::make_action_closure(ranges::bind_back(propagate_base_fn{}, std::move(fn), std::move(policy)));
  }
};

/// Implementation detail for a propagate range adaptor object with a default execution policy.
struct propagate_fn : public propagate_base_fn {
  using propagate_base_fn::operator();

  /// Overload that defines a default execution policy.
  template <
      class Range,                  //
      class StateSamplingFunction,  //
      std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(Range&& range, StateSamplingFunction fn) const -> Range& {
    return (*this)(std::execution::seq, std::forward<Range>(range), std::move(fn));
  }

  /// Overload that returns a view closure to compose with other views.
  template <class StateSamplingFunction>
  constexpr auto operator()(StateSamplingFunction fn) const {
    return ranges::make_action_closure(ranges::bind_back(propagate_fn{}, std::move(fn)));
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// can update the state in a particle range using a state transition (or sampling) function.
/**
 * This action updates particle states based on their current value and a state transition
 * (or sampling) function. Every other particle attribute (such as importance sampling weights)
 * is left unchanged.
 */
inline constexpr detail::propagate_fn propagate;

}  // namespace beluga::actions

#endif

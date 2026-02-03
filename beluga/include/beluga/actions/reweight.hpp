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

#ifndef BELUGA_ACTIONS_REWEIGHT_HPP
#define BELUGA_ACTIONS_REWEIGHT_HPP

#include <algorithm>
#include <execution>
#include <type_traits>

#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views/likelihoods.hpp>
#include <beluga/views/particles.hpp>

#include <range/v3/action/action.hpp>
#include <range/v3/view/zip.hpp>

/**
 * \file
 * \brief Implementation of the reweight range adaptor object.
 */

namespace beluga::actions {

namespace detail {

/// Internal trait to detect if a particle has a public 'likelihood' member variable.
template <typename T, typename = void>
struct has_likelihood_member : std::false_type {};

template <typename T>
struct has_likelihood_member<T, std::void_t<decltype(std::declval<T&>().likelihood)>> : std::true_type {};

/// Helper constant for the likelihood member trait.
template <typename T>
inline constexpr bool has_likelihood_member_v = has_likelihood_member<T>::value;

/// Implementation detail for a reweight range adaptor object.
struct reweight_fn {
 private:
  /// Internal core implementation for reweighting logic.
  template <class ExecutionPolicy, class Range, class LikelihoodRange>
  constexpr void apply_impl(ExecutionPolicy&& policy, Range& range, const LikelihoodRange& likelihoods) const {
    // Update particle weights using std::transform.
    auto weights = range | beluga::views::weights;
    std::transform(
        policy, std::begin(weights), std::end(weights), std::begin(likelihoods), std::begin(weights),
        [](auto weight, auto likelihood) { return weight * likelihood; });

    // Store raw likelihood if the particle type has storage for it.
    // We use ranges::views::zip here because we are performing side-effects (writing to members)
    // which std::transform is not designed for.
    using ParticleType = ranges::range_value_t<Range>;
    if constexpr (has_likelihood_member_v<ParticleType>) {
      auto zipped = ranges::views::zip(range, likelihoods);
      std::for_each(policy, std::begin(zipped), std::end(zipped), [](auto&& tuple) {
        auto& particle = std::get<0>(tuple);
        particle.likelihood = std::get<1>(tuple);
      });
    }
  }

 public:
  /// Primary overload that applies a given range of likelihoods or a model to a particle range.
  /**
   * \tparam ExecutionPolicy An [execution policy](https://en.cppreference.com/w/cpp/algorithm/execution_policy_tag_t).
   * \tparam Range An [input range](https://en.cppreference.com/w/cpp/ranges/input_range) of particles.
   * \tparam Input Either a LikelihoodRange or a Model callable.
   * \param policy The execution policy to use.
   * \param range An existing range of particles to apply this action to.
   * \param input A range of likelihood values or a sensor model to calculate them.
   */
  template <
      class ExecutionPolicy,
      class Range,
      class Input,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0,
      std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, Range& range, Input&& input) const -> Range& {
    static_assert(beluga::is_particle_range_v<Range>);
    if constexpr (ranges::range<Input>) {
      apply_impl(std::forward<ExecutionPolicy>(policy), range, input);
    } else {
      apply_impl(
          std::forward<ExecutionPolicy>(policy), range, range | beluga::views::likelihoods(std::forward<Input>(input)));
    }
    return range;
  }

  /// Convenience overload that defines a default execution policy.
  /**
   * \param range An existing range of particles to apply this action to.
   * \param input A range of likelihood values or a sensor model to calculate them.
   */
  template <
      class Range,
      class Input,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<!std::is_execution_policy_v<std::decay_t<Input>>, int> = 0>
  constexpr auto operator()(Range& range, Input&& input) const -> Range& {
    return (*this)(std::execution::seq, range, std::forward<Input>(input));
  }

  /// Re-ordering overload to support range-v3 closure argument passing.
  /**
   * This is required to bridge the gap between piped closures and the primary implementation.
   */
  template <
      class Range,
      class Input,
      class ExecutionPolicy,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0>
  constexpr auto operator()(Range&& range, Input&& input, ExecutionPolicy&& policy) const -> auto& {
    return (*this)(std::forward<ExecutionPolicy>(policy), range, std::forward<Input>(input));
  }

  /// Overload that returns a view closure to compose with other views using an execution policy.
  /**
   * \param policy The execution policy to use.
   * \param input A range of likelihood values or a sensor model to calculate them.
   */
  template <
      class ExecutionPolicy,
      class Input,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0,
      std::enable_if_t<!ranges::range<std::decay_t<ExecutionPolicy>>, int> = 0>
  constexpr auto operator()(ExecutionPolicy policy, Input input) const {
    return ranges::make_action_closure(ranges::bind_back(reweight_fn{}, std::move(input), std::move(policy)));
  }

  /// Overload that returns a view closure to compose with other views using the default execution policy.
  /**
   * \param input A range of likelihood values or a sensor model to calculate them.
   */
  template <class Input, std::enable_if_t<!std::is_execution_policy_v<std::decay_t<Input>>, int> = 0>
  constexpr auto operator()(Input input) const {
    return ranges::make_action_closure(ranges::bind_back(reweight_fn{}, std::move(input)));
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// updates the weights in a particle range using a sensor model or a likelihood range.
/**
 * This action updates particle weights by importance weight multiplication.
 * If the particle type has a public `likelihood` member, the raw likelihood values
 * used for the update are also stored in the particles.
 */
inline constexpr detail::reweight_fn reweight;

}  // namespace beluga::actions

#endif

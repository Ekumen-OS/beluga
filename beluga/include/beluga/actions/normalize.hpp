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
#include <optional>

#include <range/v3/action/action.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/common.hpp>

#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views/particles.hpp>

/**
 * \file
 * \brief Implementation of the normalize range adaptor object.
 */

namespace beluga::actions {

namespace detail {

/// \cond detail

template <class ExecutionPolicy = std::execution::sequenced_policy>
struct normalize_closure {
 public:
  static_assert(std::is_execution_policy_v<ExecutionPolicy>);

  constexpr normalize_closure() noexcept : policy_{std::execution::seq} {}

  constexpr explicit normalize_closure(ExecutionPolicy policy) : policy_{std::move(policy)} {}

  constexpr explicit normalize_closure(double factor) noexcept : policy_{std::execution::seq}, factor_{factor} {}

  constexpr normalize_closure(ExecutionPolicy policy, double factor) : policy_{std::move(policy)}, factor_{factor} {}

  template <class Range>
  constexpr auto operator()(Range& range) const {
    static_assert(ranges::forward_range<Range>);

    auto weights = std::invoke([&range]() {
      if constexpr (beluga::is_particle_range_v<Range>) {
        return range | beluga::views::weights | ranges::views::common;
      } else {
        return range | ranges::views::common;
      }
    });

    const double factor = std::invoke([this, weights]() {
      if (factor_.has_value()) {
        return factor_.value();
      }

      return ranges::accumulate(weights, 0.0);  // The default normalization factor is the total sum of weights.
    });

    if (std::abs(factor - 1.0) < std::numeric_limits<double>::epsilon()) {
      return range;  // No change.
    }

    std::transform(
        policy_,          //
        weights.begin(),  //
        weights.end(),    //
        weights.begin(),  //
        [factor](const auto w) { return w / factor; });

    return range;
  }

 private:
  ExecutionPolicy policy_{};
  std::optional<double> factor_;
};

struct normalize_fn {
  template <
      class ExecutionPolicy,
      class Range,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0,
      std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, Range& range, double factor) const -> Range& {
    return normalize_closure{std::forward<ExecutionPolicy>(policy), factor}(range);
  }

  template <
      class ExecutionPolicy,
      class Range,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0,
      std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, Range& range) const -> Range& {
    return normalize_closure{std::forward<ExecutionPolicy>(policy)}(range);
  }

  template <class Range, std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(Range& range, double factor) const -> Range& {
    return normalize_closure{factor}(range);
  }

  template <class Range, std::enable_if_t<ranges::range<Range>, int> = 0>
  constexpr auto operator()(Range& range) const -> Range& {
    return normalize_closure{}(range);
  }

  template <class ExecutionPolicy, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, double factor) const {
    return ranges::actions::action_closure{normalize_closure{std::forward<ExecutionPolicy>(policy), factor}};
  }

  template <class ExecutionPolicy, std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy) const {
    return ranges::actions::action_closure{normalize_closure{std::forward<ExecutionPolicy>(policy)}};
  }

  constexpr auto operator()(double factor) const { return ranges::actions::action_closure{normalize_closure{factor}}; }

  constexpr auto operator()() const { return ranges::actions::action_closure{normalize_closure{}}; }
};

/// \endcond

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
inline constexpr detail::normalize_fn normalize;

}  // namespace beluga::actions

#endif

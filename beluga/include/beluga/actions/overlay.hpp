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

#ifndef BELUGA_ACTIONS_OVERLAY_HPP
#define BELUGA_ACTIONS_OVERLAY_HPP

#include <algorithm>
#include <execution>

#include <range/v3/action/action.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/transform.hpp>

/**
 * \file
 * \brief Implementation of the overlay range adaptor object
 */
namespace beluga::actions {

namespace detail {

/// Implementation detail for an overlay range adaptor object.
struct overlay_base_fn {
  /// Overload that implements an overlay of a value in a range.
  /**
   * \tparam ExecutionPolicy An [execution policy](https://en.cppreference.com/w/cpp/algorithm/execution_policy_tag_t).
   * \tparam Range An [input range](https://en.cppreference.com/w/cpp/ranges/input_range).
   * \tparam MaskRange An [input range](https://en.cppreference.com/w/cpp/ranges/input_range).
   * \param policy The execution policy to use.
   * \param range An existing range to apply this action to.
   * \param mask The mask where the values will be overlaid.
   * \param mask_value The value to be overlaid.
   */
  template <
      class ExecutionPolicy,
      class Range,
      class MaskRange,
      class Mask,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, Range& range, MaskRange&& mask, Mask&& mask_value) const
      -> Range& {
    auto map = range | ranges::views::common;

    std::transform(
        policy,            //
        std::begin(map),   //
        std::end(map),     //
        std::begin(mask),  //
        std::begin(map),   //
        [&mask_value](const auto& base_value, bool flag) { return flag ? mask_value : base_value; });

    return range;
  }

  /// Overload that re-orders arguments from an action closure.
  template <
      class ExecutionPolicy,
      class Range,
      class MaskRange,
      class Mask,
      std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>, int> = 0,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(Range&& range, MaskRange&& mask, Mask&& mask_value, ExecutionPolicy policy) const
      -> Range& {
    return (*this)(
        std::move(policy), std::forward<Range>(range), std::forward<MaskRange>(mask), std::forward<Mask>(mask_value));
  }

  /// Overload that returns an action closure to compose with other actions.
  template <
      class ExecutionPolicy,
      class MaskRange,
      class Mask,
      std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>, int> = 0,
      std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(ExecutionPolicy policy, MaskRange&& mask, Mask&& mask_value) const {
    return ranges::make_action_closure(ranges::bind_back(
        overlay_base_fn{}, std::forward<MaskRange>(mask), std::forward<Mask>(mask_value), std::move(policy)));
  }
};

/// Implementation detail for an overlay range adaptor object with a default execution policy.
struct overlay_fn : public overlay_base_fn {
  using overlay_base_fn::operator();

  /// Overload that defines a default execution policy.
  template <
      class Range,
      class MaskRange,
      class Mask,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(Range&& range, MaskRange&& mask, Mask&& mask_value) const -> Range& {
    return (*this)(
        std::execution::seq, std::forward<Range>(range), std::forward<MaskRange>(mask), std::forward<Mask>(mask_value));
  }

  /// Overload that returns an action closure to compose with other actions.
  template <class MaskRange, class Mask, std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(MaskRange&& mask, Mask&& mask_value) const {
    return ranges::make_action_closure(
        ranges::bind_back(overlay_fn{}, std::forward<MaskRange>(mask), std::forward<Mask>(mask_value)));
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// can overlay a range of values (or a range of particles).
/**
 * The `overlay` range adaptor allows to overlay the values of the range that match a mask.
 * All the values are overlaid for a given value.
 */
inline constexpr ranges::actions::action_closure<detail::overlay_fn> overlay;
}  // namespace beluga::actions

#endif

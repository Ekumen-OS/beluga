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

/**
 * \file
 * \brief Implementation of the overlay range adaptor object
 */
namespace beluga::actions {

namespace detail {

///\cond detail

template <class MaskRange, class MaskValue, class ExecutionPolicy = std::execution::sequenced_policy>
struct overlay_closure {
 public:
  static_assert(std::is_execution_policy_v<ExecutionPolicy>);
  static_assert(ranges::range<MaskRange>);

  constexpr overlay_closure(MaskRange mask_range, MaskValue mask_value)
      : policy_{std::execution::seq}, mask_range_{std::move(mask_range)}, mask_value_{std::move(mask_value)} {}

  constexpr explicit overlay_closure(ExecutionPolicy policy, MaskRange mask_range, MaskValue mask_value)
      : policy_{std::move(policy)}, mask_range_{std::move(mask_range)}, mask_value_{std::move(mask_value)} {}

  template <class Range>
  constexpr auto operator()(Range& range) const -> Range& {
    static_assert(ranges::range<Range>);
    auto common_range = range | ranges::views::common;
    const auto converted_mask_value = static_cast<ranges::range_value_t<Range>>(mask_value_);

    std::transform(
        policy_,                   //
        std::begin(common_range),  //
        std::end(common_range),    //
        std::begin(mask_range_),   //
        std::begin(common_range),  //
        [&converted_mask_value](const auto& base_value, bool flag) {
          return flag ? converted_mask_value : base_value;
        });

    return range;
  }

 private:
  ExecutionPolicy policy_{};
  MaskRange mask_range_{};
  MaskValue mask_value_{};
};

struct overlay_fn {
  template <
      class ExecutionPolicy,
      class Range,
      class MaskRange,
      class MaskValue,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, Range& range, MaskRange&& mask_range, MaskValue&& mask_value)
      const -> Range& {
    return overlay_closure{
        std::forward<ExecutionPolicy>(policy), std::forward<MaskRange>(mask_range),
        std::forward<MaskValue>(mask_value)}(range);
  }

  template <
      class Range,
      class MaskRange,
      class MaskValue,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(Range& range, MaskRange&& mask_range, MaskValue&& mask_value) const -> Range& {
    return overlay_closure{std::forward<MaskRange>(mask_range), std::forward<MaskValue>(mask_value)}(range);
  }

  template <
      class ExecutionPolicy,
      class MaskRange,
      class MaskValue,
      std::enable_if_t<std::is_execution_policy_v<std::decay_t<ExecutionPolicy>>, int> = 0,
      std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, MaskRange&& mask_range, MaskValue&& mask_value) const {
    return ranges::actions::action_closure{overlay_closure{
        std::forward<ExecutionPolicy>(policy), std::forward<MaskRange>(mask_range),
        std::forward<MaskValue>(mask_value)}};
  }

  template <class MaskRange, class MaskValue, std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(MaskRange&& mask_range, MaskValue&& mask_value) const {
    return ranges::actions::action_closure{
        overlay_closure{std::forward<MaskRange>(mask_range), std::forward<MaskValue>(mask_value)}};
  }
};

/// \endcond

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

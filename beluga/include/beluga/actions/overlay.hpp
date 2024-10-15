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

///\cond detail

template <class MaskRange, class Mask, class ExecutionPolicy = std::execution::sequenced_policy>
struct overlay_closure {
 public:
  static_assert(std::is_execution_policy_v<ExecutionPolicy>);
  static_assert(ranges::range<MaskRange>);

  constexpr overlay_closure(MaskRange mask, Mask mask_value)
      : policy_{std::execution::seq}, mask_(std::move(mask)), mask_value_(std::move(mask_value)) {}

  constexpr explicit overlay_closure(ExecutionPolicy policy, MaskRange mask, Mask mask_value)
      : policy_(std::move(policy)), mask_(std::move(mask)), mask_value_(std::move(mask_value)) {}

  template <class Range>
  constexpr auto operator()(Range& range) const -> Range& {
    static_assert(ranges::range<Range>);
    auto map = range | ranges::views::common;
    const auto converted_mask_value = static_cast<ranges::range_value_t<Range>>(mask_value_);

    std::transform(
        policy_,            //
        std::begin(map),    //
        std::end(map),      //
        std::begin(mask_),  //
        std::begin(map),    //
        [&converted_mask_value](const auto& base_value, bool flag) {
          return flag ? converted_mask_value : base_value;
        });

    return range;
  }

 private:
  ExecutionPolicy policy_{};
  MaskRange mask_{};
  Mask mask_value_{};
};

struct overlay_fn {
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
    return overlay_closure{
        std::forward<ExecutionPolicy>(policy), std::forward<MaskRange>(mask), std::forward<Mask>(mask_value)}(range);
  }

  template <
      class Range,
      class MaskRange,
      class Mask,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(Range& range, MaskRange&& mask, Mask&& mask_value) const -> Range& {
    return overlay_closure{std::forward<MaskRange>(mask), std::forward<Mask>(mask_value)}(range);
  }

  template <
      class ExecutionPolicy,
      class MaskRange,
      class Mask,
      std::enable_if_t<std::is_execution_policy_v<ExecutionPolicy>, int> = 0,
      std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(ExecutionPolicy&& policy, MaskRange&& mask, Mask&& mask_value) const {
    return ranges::actions::action_closure{overlay_closure{
        std::forward<ExecutionPolicy>(policy), std::forward<MaskRange>(mask), std::forward<Mask>(mask_value)}};
  }

  template <class MaskRange, class Mask, std::enable_if_t<ranges::range<MaskRange>, int> = 0>
  constexpr auto operator()(MaskRange&& mask, Mask&& mask_value) const {
    return ranges::actions::action_closure{
        overlay_closure{std::forward<MaskRange>(mask), std::forward<Mask>(mask_value)}};
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

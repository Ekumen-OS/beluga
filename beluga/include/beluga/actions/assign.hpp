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

#ifndef BELUGA_ACTIONS_ASSIGN_HPP
#define BELUGA_ACTIONS_ASSIGN_HPP

#include <range/v3/action/action.hpp>
#include <range/v3/functional/bind_back.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/view.hpp>

namespace beluga::actions {

/// \cond

template <class T, class = void>
struct is_range_closure : std::false_type {};

template <class T>
struct is_range_closure<ranges::actions::action_closure<T>> : std::true_type {};

template <class T>
struct is_range_closure<ranges::views::view_closure<T>> : std::true_type {};

template <class T>
inline constexpr bool is_range_closure_v = is_range_closure<T>::value;

/// \endcond

namespace detail {

/// Implementation detail for an assign range adaptor object.
struct assign_fn {
  /// Overload that implements the assign algorithm.
  template <
      class Range,
      class Fn,
      std::enable_if_t<ranges::range<Range>, int> = 0,
      std::enable_if_t<is_range_closure_v<Fn>, int> = 0>
  constexpr auto operator()(Range& range, Fn fn) const -> Range& {
    auto&& view = fn(range);
    if constexpr (!std::is_same_v<Range, std::decay_t<decltype(view)>>) {
      // If the result of invoking the closure is not the range itself,
      // then we need to convert the view and assign it to the input range.
      range = std::move(view) | ranges::to<Range>;
    }
    return range;
  }

  /// Hidden friend operator overload that enables action / view composition.
  /**
   * Enables the following expressions:
   *   1) view_closure | assign
   *   2) action_closure | view_closure | assign
   *   3) action_closure | assign
   *
   * 1) Will create an action closure that can eagerly invoke `view_closure` and assign
   *    the resulting range to the input range.
   * 2) Will create a new action closure that can invoke `action_closure`, eagerly invoke
   *    `view_closure`, and assign the resulting range to the input range.
   * 3) Technically a no-op, it will just invoke the `action_closure`.
   */
  template <class Fn, std::enable_if_t<is_range_closure_v<Fn>, int> = 0>
  friend constexpr auto operator|(Fn fn, assign_fn) {
    return ranges::make_action_closure(ranges::bind_back(assign_fn{}, std::move(fn)));
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// can converts a view closure into an action closure.
/**
 * This can be appended to any view closure to evaluate it eagerly and assign the result
 * to a given range, effectively converting any view into an action.
 *
 * The assignment of elements into the container may involve copy which can be less efficient than
 * move because lvalue references are produced during the indirection call. Users can opt-in to use
 * `ranges::views::move` to adapt the range in order for their elements to always produce an rvalue
 * reference during the indirection call which implies move. In this case, make sure that the input
 * views don't require accessing the elements in the adapted range more than once.
 */
inline constexpr detail::assign_fn assign;

/// Operator overload that appends assign to any range closure.
template <
    class Range,
    class Fn,
    std::enable_if_t<ranges::range<Range>, int> = 0,
    std::enable_if_t<is_range_closure_v<Fn>, int> = 0>
constexpr auto operator<<=(Range& range, Fn fn) -> Range& {
  return range |= std::move(fn) | beluga::actions::assign;
}

}  // namespace beluga::actions

// Make the assign operator overload findable by ADL for existing range adaptor objects:

namespace beluga::views {
using beluga::actions::operator<<=;
}

namespace ranges::views {
using beluga::actions::operator<<=;
}

namespace ranges::actions {
using beluga::actions::operator<<=;
}

#endif

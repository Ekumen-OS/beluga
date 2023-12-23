// Copyright 2023 Ekumen, Inc.
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

#ifndef BELUGA_VIEWS_TAKE_EVENLY_HPP
#define BELUGA_VIEWS_TAKE_EVENLY_HPP

#include <range/v3/view/stride.hpp>
#include <range/v3/view/take.hpp>

/**
 * \file
 * \brief Implementation of a take_evenly range adaptor object.
 */

namespace beluga::views {

namespace detail {

/// Implementation detail for a take_evenly range adaptor object.
struct take_evenly_fn {  // NOLINT(readability-identifier-naming)
  /// Overload that implements the take_evenly algorithm.
  /**
   * \tparam Range A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range).
   * \param range Source range from where to take elements.
   * \param count Number of elements to take.
   *
   * If `count` or the range size are zero, it returns an empty range.
   * If `count` is greater than the range size, it returns all the elements.
   * The step size is computed to always include the last element of the range.
   */
  template <class Range>
  constexpr auto operator()(Range&& range, std::size_t count) const {
    // Note: `stride` doesn't support step == 0.
    const std::size_t size = ranges::size(range);
    if (count == 0 || size == 0) {
      return range | ranges::views::take(0) | ranges::views::stride(1);
    } else if (count == 1) {
      return range | ranges::views::take(1) | ranges::views::stride(1);
    } else {
      // Make sure that the last element of the range is included and that the minimum step size is 1.
      const std::size_t step = std::max(1UL, (size - 1UL) / (count - 1UL));
      return range | ranges::views::take(size) | ranges::views::stride(step);
    }
  }

  /// Overload that returns a view closure to compose with other views.
  /**
   * \param count Number of elements to take.
   *
   * If `count` or the range size are zero, it returns an empty range.
   * If `count` is greater than the range size, it returns all the elements.
   * The step size is computed to always include the last element of the range.
   */
  constexpr auto operator()(std::size_t count) const {
    return ranges::make_view_closure(ranges::bind_back(take_evenly_fn{}, count));
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// will take elements evenly spaced over a range.
/**
 * Given a source range and an integral count, returns a range consisting of `count`
 * elements evenly spaced over the source range.
 * If `count` or the range size are zero, it returns an empty range.
 * If `count` is greater than the range size, it returns all the elements.
 * The step size is computed to always include the last element of the range.
 */
inline constexpr detail::take_evenly_fn take_evenly;  // NOLINT(readability-identifier-naming)

}  // namespace beluga::views

#endif

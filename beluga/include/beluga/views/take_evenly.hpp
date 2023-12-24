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

#include <beluga/views/elements.hpp>

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>

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
   * The first and last elements of the range are always included.
   */
  template <class Range>
  constexpr auto operator()(Range&& range, std::size_t count) const {
    const std::size_t size = ranges::size(range);

    const auto filter_function = [size, count](const auto& pair) {
      if ((size == 0UL) || (count == 0UL)) {
        return false;
      }

      if (count > size) {
        return true;
      }

      const auto [index, _] = pair;
      if (count == 1UL) {
        return index == 0UL;
      }

      if ((index == 0UL) || (index == size - 1UL)) {
        return true;
      }

      const std::size_t m0 = (index - 1UL) * (count - 1UL) / (size - 1UL);
      const std::size_t m1 = index * (count - 1UL) / (size - 1UL);
      return m0 != m1;
    };

    return ranges::views::enumerate(range) | ranges::views::filter(filter_function) | beluga::views::elements<1>;
  }

  /// Overload that returns a view closure to compose with other views.
  /**
   * \param count Number of elements to take.
   *
   * If `count` or the range size are zero, it returns an empty range.
   * If `count` is greater than the range size, it returns all the elements.
   * The first and last elements of the range are always included.
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
 * The first and last elements of the range are always included.
 */
inline constexpr detail::take_evenly_fn take_evenly;  // NOLINT(readability-identifier-naming)

}  // namespace beluga::views

#endif

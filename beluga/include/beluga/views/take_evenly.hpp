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

/**
 * \file
 * \brief Implementation of a take_evenly range adaptor object.
 */

namespace beluga::views {

namespace detail {

/// Implementation detail for a take_evenly range adaptor object.
struct take_evenly_fn {  // NOLINT(readability-identifier-naming)
  /// Overload that implements the take_evenly algorithm.
  template <class Range>
  constexpr auto operator()(Range&& range, std::size_t max_size) const {
    const std::size_t size = ranges::size(range);
    const std::size_t step = (max_size > 1UL)    ? std::max(1UL, (size - 1UL) / (max_size - 1UL))
                             : (max_size == 1UL) ? size
                                                 : 1UL;
    return range | ranges::views::stride(step);
  }

  /// Overload that returns a view closure to compose with other views.
  constexpr auto operator()(std::size_t max_size) const {
    return ranges::make_view_closure(ranges::bind_back(take_evenly_fn{}, max_size));
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that,
/// given a maximum size, will take elements evenly distributed over a range.
inline constexpr detail::take_evenly_fn take_evenly;  // NOLINT(readability-identifier-naming)

}  // namespace beluga::views

#endif

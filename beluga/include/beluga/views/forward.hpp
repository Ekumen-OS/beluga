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

#ifndef BELUGA_VIEWS_FORWARD_HPP
#define BELUGA_VIEWS_FORWARD_HPP

#include <beluga/definitions.hpp>

#include <range/v3/view/adaptor.hpp>

/**
 * \file
 * \brief Implementation of a forward range adaptor object.
 */

namespace beluga::views {

namespace detail {

/// Implementation of a forward view adaptor.
/**
 * Restricts a range to model the forward range concept or lower.
 */
template <class Range>
struct forward_view : public ranges::view_adaptor<forward_view<Range>, Range, ranges::range_cardinality<Range>::value> {
 public:
  /// Default constructor.
  forward_view() = default;

  /// Construct the view from an existing range.
  constexpr explicit forward_view(Range range) : forward_view::view_adaptor{std::move(range)} {}

 private:
  // `ranges::range_access` needs access to the cursor members.
  friend ranges::range_access;

  /// Adaptor subclass that just deletes operations.
  struct adaptor : public ranges::adaptor_base {
    adaptor() = default;

    void prev(ranges::iterator_t<Range>& it) = delete;
    void advance() = delete;
    void distance_to() = delete;
  };

  /// Return the adaptor for the begin iterator.
  [[nodiscard]] constexpr auto begin_adaptor() const { return adaptor{}; }
};

/// Implementation detail for a forward range adaptor object.
struct forward_fn {
  /// Overload that adapts an existing range.
  template <class Range>
  constexpr auto operator()(Range&& range) const {
    return forward_view{ranges::views::all(std::forward<Range>(range))};
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// restricts an existing range to model the forward range concept or lower.
inline constexpr ranges::views::view_closure<detail::forward_fn> forward;

}  // namespace beluga::views

namespace ranges {

/// \cond

/// Enable borrowed range specialization.
/**
 * A function can take this range by value and return iterators obtained from it without danger of dangling,
 * as long as the input range is also a borrowed range.
 */
template <class Range>
inline constexpr bool enable_borrowed_range<beluga::views::detail::forward_view<Range>> = enable_borrowed_range<Range>;

/// \endcond

}  // namespace ranges

#endif

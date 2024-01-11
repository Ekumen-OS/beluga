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

#ifndef BELUGA_VIEWS_ZIP_HPP
#define BELUGA_VIEWS_ZIP_HPP

#include <tuple>

#include <range/v3/view/zip.hpp>

/**
 * \file
 * \brief Implementation of a zip range adaptor object.
 */

namespace beluga::views {

namespace detail {

/// Utility type to adapt the zip output.
struct as_common_tuple_indirect_fn {
  /// Reference overload.
  template <class... Its>
  constexpr auto operator()(const Its&... its) const  //
      noexcept((noexcept(ranges::iter_reference_t<Its>(*its)) && ...)) {
    return ranges::common_tuple<ranges::iter_reference_t<Its>...>{*its...};
  }

  /// Move overload.
  template <class... Its>
  constexpr auto operator()(ranges::move_tag, const Its&... its) const
      noexcept((noexcept(ranges::iter_rvalue_reference_t<Its>(ranges::iter_move(its))) && ...)) {
    return ranges::common_tuple<ranges::iter_rvalue_reference_t<Its>...>{ranges::iter_move(its)...};
  }

  /// Copy overload.
  /**
   * This is needed for `ranges_value_t` to return `std::tuple` when this object is passed to a `zip_with_view`.
   */
  template <class... Its>
  constexpr auto operator()(ranges::copy_tag, Its...) const -> std::tuple<ranges::iter_value_t<Its>...> {
    RANGES_EXPECT(false);
  }
};

/// Implementation detail for a zip range adaptor object.
struct zip_fn {
  /// Overload that implements the zip_view algorithm.
  template <class... Ranges>
  constexpr auto operator()(Ranges&&... ranges) const {
    return ranges::views::iter_zip_with(as_common_tuple_indirect_fn{}, std::forward<Ranges>(ranges)...);
  }
};

}  // namespace detail

/// Given N ranges, return a new range where the Mth element is a tuple of the Mth elements of all N ranges.
/**
 * Unlike `ranges::views::zip`, iterators always dereference into tuples, not pairs.
 * Other than that, they are both equivalent views.
 */
inline constexpr detail::zip_fn zip;

}  // namespace beluga::views

#endif

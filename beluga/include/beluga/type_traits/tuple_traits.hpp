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

#ifndef BELUGA_TYPE_TRAITS_TUPLE_TRAITS_HPP
#define BELUGA_TYPE_TRAITS_TUPLE_TRAITS_HPP

#include <tuple>
#include <type_traits>

/**
 * \file
 * \brief Implementation of traits for tuple-like types.
 */

namespace beluga {

namespace detail {

/// Constant value to return when the index was not found.
static constexpr std::size_t kTupleIndexNotFound = static_cast<std::size_t>(-1);

/// Constant value to return when there are multiple indices that match the input type.
static constexpr std::size_t kTupleIndexAmbiguous = static_cast<std::size_t>(-2);

/// Help method that finds a tuple element index that matches an input type.
/**
 * Tuple types will be decayed before comparing, so tuple_index_helper<T, Args...>
 * can find T&, const T& and other variants in the tuple types.
 */
template <class T, class... Args>
constexpr std::size_t tuple_index_helper() noexcept {
  constexpr bool kMatches[sizeof...(Args)] =  // NOLINT(modernize-avoid-c-arrays)
      {std::is_same<T, std::decay_t<Args>>::value...};
  std::size_t selected = kTupleIndexNotFound;
  for (std::size_t i = 0; i < sizeof...(Args); ++i) {
    if (kMatches[i]) {
      if (selected == kTupleIndexNotFound) {
        selected = i;
      } else {
        return kTupleIndexAmbiguous;
      }
    }
  }
  return selected;
}

}  // namespace detail

/// Meta-function that returns the tuple index of the element whose type is T.
template <class T, class TupleLike>
struct tuple_index {};

/// `tuple_index` specialization for tuples.
/**
 * Fails to compile unless the tuple has exactly one element of that type.
 */
template <class T, template <class...> class TupleLike, class... Args>
struct tuple_index<T, TupleLike<Args...>> {
  /// `tuple_index` return value.
  static constexpr std::size_t value = detail::tuple_index_helper<T, Args...>();
  static_assert(value != detail::kTupleIndexNotFound);
  static_assert(value != detail::kTupleIndexAmbiguous);
};

/// Convenience template variable for `tuple_index`.
template <class T, class TupleLike>
inline constexpr std::size_t tuple_index_v = tuple_index<T, TupleLike>::value;

/// Returns element of a tuple like object whose type is T (or a possibly const reference to T).
template <class T, class TupleLike>
constexpr decltype(auto) element_of_type(TupleLike&& tuple) noexcept {
  constexpr std::size_t kIndex = tuple_index_v<T, std::decay_t<TupleLike>>;
  return std::get<kIndex>(std::forward<TupleLike>(tuple));
}

}  // namespace beluga

#endif

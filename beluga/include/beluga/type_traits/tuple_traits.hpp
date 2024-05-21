// Copyright 2023-2024 Ekumen, Inc.
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
#include <utility>

/**
 * \file
 * \brief Implementation of traits for tuple-like types.
 */

namespace beluga {

namespace detail {

/// \cond

template <typename, typename = void>
struct is_complete : std::false_type {};

template <typename T>
struct is_complete<T, std::void_t<decltype(sizeof(T))>> : std::true_type {};

/// \endcond

}  // namespace detail

/// Meta-function that returns true if T is a tuple-like type.
/**
 * tuple-like types implement the tuple protocol.
 * See https://en.cppreference.com/w/cpp/utility/tuple/tuple-like
 */
template <typename T>
struct is_tuple_like : detail::is_complete<std::tuple_size<std::decay_t<T>>> {};

/// Convenience template variable for `is_tuple_like`.
template <class T>
inline constexpr bool is_tuple_like_v = is_tuple_like<T>::value;

/// Meta-function that computes a common tuple type given two tuple-like types T and U.
/**
 * Equivalent to `common_types_t` specialization for tuple-like types in C++23.
 * See https://en.cppreference.com/w/cpp/utility/tuple/common_type.
 */
template <
    typename T,
    typename U,
    typename Is = std::enable_if_t<
        std::tuple_size_v<std::decay_t<T>> == std::tuple_size_v<std::decay_t<U>>,
        std::make_index_sequence<std::tuple_size_v<std::decay_t<T>>>>>
struct common_tuple_type;

/// `common_tuple_type` specialization for `std::index_sequence`.
template <typename T, typename U, std::size_t... I>
struct common_tuple_type<T, U, std::index_sequence<I...>> {
  /// Common tuple type based on an std::tuple.
  using type = std::tuple<
      std::common_type_t<std::tuple_element_t<I, std::decay_t<T>>, std::tuple_element_t<I, std::decay_t<U>>>...>;
};

/// Convenience template type alias for `common_tuple_type`.
template <typename T, typename U>
using common_tuple_type_t = typename common_tuple_type<T, U>::type;

/// Meta-function that checks for the existence of a common tuple type.
template <typename, typename, typename = void>
struct has_common_tuple_type : std::false_type {};

/// `has_common_tuple_type` specialization for tuple-like types T and U for which a common tuple type exists.
template <typename T, typename U>
struct has_common_tuple_type<T, U, std::void_t<common_tuple_type_t<T, U>>> : std::true_type {};

/// Convenience template variable for `has_common_tuple_type`.
template <typename T, typename U>
inline constexpr bool has_common_tuple_type_v = has_common_tuple_type<T, U>::value;

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
      {std::is_same_v<T, std::decay_t<Args>>...};
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

/// Help method that returns true if a tuple element index that matches an input type is found.
template <class T, class... Args>
constexpr bool tuple_index_found() noexcept {
  constexpr std::size_t kIndex = tuple_index_helper<T, Args...>();
  return kIndex != kTupleIndexNotFound && kIndex != kTupleIndexAmbiguous;
}

}  // namespace detail

/// Meta-function that returns the tuple index of the element whose type is T.
template <class T, class TupleLike, class = void>
struct tuple_index;

/// `tuple_index` specialization for tuples.
template <class T, template <class...> class TupleLike, class... Args>
struct tuple_index<
    T,
    TupleLike<Args...>,
    std::enable_if_t<is_tuple_like_v<std::decay_t<TupleLike<Args...>>> && detail::tuple_index_found<T, Args...>()>>
    : std::integral_constant<std::size_t, detail::tuple_index_helper<T, Args...>()> {};

/// Convenience template variable for `tuple_index`.
template <class T, class TupleLike>
inline constexpr std::size_t tuple_index_v = tuple_index<T, TupleLike>::value;

/// Convenience template type alias for `tuple_index`.
template <class T, class TupleLike>
using tuple_index_t = typename tuple_index<T, TupleLike>::type;

/// Meta-function that returns true if there is a single element of type T in the tuple-like type.
template <class T, class TupleLike, class = void>
struct has_single_element : std::false_type {};

/// `has_single_element` specialization for tuples.
template <class T, template <class...> class TupleLike, class... Args>
struct has_single_element<
    T,
    TupleLike<Args...>,
    std::enable_if_t<is_tuple_like_v<std::decay_t<TupleLike<Args...>>> && detail::tuple_index_found<T, Args...>()>>
    : std::true_type {};

/// Convenience template variable for `has_single_element`.
template <class T, class TupleLike>
inline constexpr bool has_single_element_v = has_single_element<T, TupleLike>::value;

/// Returns element of a tuple like object whose type is T (or a possibly const reference to T).
template <class T, class TupleLike>
constexpr decltype(auto) element(TupleLike&& tuple) noexcept {
  constexpr std::size_t kIndex = tuple_index_v<T, std::decay_t<TupleLike>>;
  return std::get<kIndex>(std::forward<TupleLike>(tuple));
}

/// Meta-function that decays a tuple like type and its members.
template <class T, class = void>
struct decay_tuple_like;

/// `decay_tuple_like` specialization for tuples.
template <template <class...> class TupleLike, class... Args>
struct decay_tuple_like<TupleLike<Args...>, std::enable_if_t<is_tuple_like_v<std::decay_t<TupleLike<Args...>>>>> {
  /// Return type.
  using type = std::decay_t<TupleLike<std::decay_t<Args>...>>;
};

/// Convenience template type alias for `decay_tuple_like`.
template <class T>
using decay_tuple_like_t = typename decay_tuple_like<T>::type;

}  // namespace beluga

#endif

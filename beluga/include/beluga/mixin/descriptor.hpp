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

#ifndef BELUGA_MIXIN_DESCRIPTOR_HPP
#define BELUGA_MIXIN_DESCRIPTOR_HPP

#include <type_traits>

#include <beluga/mixin/utility.hpp>

namespace beluga::mixin {

template <template <class> class Mixin>
struct tag {
  template <class Base>
  using mixin = Mixin<Base>;
};

template <template <class> class Mixin, class Param>
struct descriptor {
  template <class Base>
  using mixin = Mixin<Base>;

  Param params;
};

namespace detail {

template <typename T>
struct is_tag_impl : public std::false_type {};

template <template <class> class Mixin>
struct is_tag_impl<tag<Mixin>> : public std::true_type {};

template <typename T>
struct is_descriptor_impl : public std::false_type {};

template <template <class> class Mixin, class Param>
struct is_descriptor_impl<descriptor<Mixin, Param>> : public std::true_type {};

}  // namespace detail

template <typename T>
struct is_tag {
  static constexpr bool value = detail::is_tag_impl<std::decay_t<T>>::value;  // NOLINT
};

template <typename T>
using is_not_tag = std::negation<is_tag<T>>;

template <typename T>
struct is_descriptor {
  static constexpr bool value = detail::is_descriptor_impl<std::decay_t<T>>::value;  // NOLINT
};

template <typename T>
using is_not_descriptor = std::negation<is_descriptor<T>>;

template <typename T>
using is_descriptor_or_tag = std::disjunction<is_descriptor<T>, is_tag<T>>;

template <typename T>
inline constexpr bool is_tag_v = is_tag<T>::value;  // NOLINT

template <typename T>
inline constexpr bool is_descriptor_v = is_descriptor<T>::value;  // NOLINT

template <typename T>
inline constexpr bool is_descriptor_or_tag_v = is_descriptor_or_tag<T>::value;  // NOLINT

template <template <template <class> class...> class Base, class... Types>
struct mixin_from_descriptors {
  using type = Base<std::decay_t<Types>::template mixin...>;

  static_assert((is_descriptor_or_tag_v<Types> && ...), "Invalid mixin descriptors or tags");
};

template <template <template <class> class...> class Base, class... Types>
struct mixin_from_descriptors<Base, beluga::mixin::list<Types...>> {
  using type = Base<std::decay_t<Types>::template mixin...>;

  static_assert((is_descriptor_or_tag_v<Types> && ...), "Invalid mixin descriptors or tags");
};

template <template <template <class> class...> class Base, class... Descriptors>
using mixin_from_descriptors_t = typename mixin_from_descriptors<Base, Descriptors...>::type;

}  // namespace beluga::mixin

#endif

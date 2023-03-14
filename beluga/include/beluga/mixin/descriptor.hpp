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

/**
 * \file
 * \brief Implementation of descriptors and their traits.
 */

namespace beluga::mixin {

/// Primary template for a mixin descriptor.
/**
 * \tparam Mixin A mixin template class.
 * \tparam Params A variadic list of parameter types.
 */
template <template <class> class Mixin, class... Params>
struct descriptor;

/// Partial template specialization for mixin descriptor with no parameters.
/**
 * \tparam Mixin A mixin template class.
 */
template <template <class> class Mixin>
struct descriptor<Mixin> {
  /// The mixin template type.
  template <class Base>
  using mixin = Mixin<Base>;
};

/// Partial template specialization for mixin descriptor with a single parameter.
/**
 * \tparam Mixin A mixin template class.
 * \tparam Param The parameter type.
 */
template <template <class> class Mixin, class Param>
struct descriptor<Mixin, Param> {
  /// The mixin template type.
  template <class Base>
  using mixin = Mixin<Base>;

  /// The parameter object.
  Param params;
};

/// \cond detail

namespace detail {

template <class T, typename Enable = void>
struct has_params_impl : public std::false_type {};

template <class T>
struct has_params_impl<T, std::void_t<decltype(std::declval<T&>().params)>> : public std::true_type {};

template <typename T>
struct is_descriptor_impl : public std::false_type {};

template <template <class> class Mixin>
struct is_descriptor_impl<descriptor<Mixin>> : public std::true_type {};

template <template <class> class Mixin, class Param>
struct is_descriptor_impl<descriptor<Mixin, Param>> : public std::true_type {};

}  // namespace detail

/// \endcond

/// Trait to detect that a given type has a params member.
/**
 * If `T` (after decay) has a `params` variable member,
 * provides a member constant value equal to `true`. For any other type, value is `false`.
 *
 * \tparam T A type to check.
 */
template <typename T>
struct has_params {
  /// Member constant value.
  static constexpr bool value = detail::has_params_impl<std::decay_t<T>>::value;  // NOLINT
};

/// Helper variable template to detect that a given type has a params member.
template <typename T>
inline constexpr bool has_params_v = has_params<T>::value;  // NOLINT

/// Type trait to detect that a given type is a descriptor.
/**
 * If `T` (after decay) is an instantiation of \ref beluga::mixin::descriptor,
 * provides a member constant value equal to `true`. For any other type, value is `false`.
 *
 * \tparam T A type to check.
 */
template <typename T>
struct is_descriptor {
  /// Member constant value.
  static constexpr bool value = detail::is_descriptor_impl<std::decay_t<T>>::value;  // NOLINT
};

/// Helper type alias to detect that a given type is not a descriptor.
template <typename T>
using is_not_descriptor = std::negation<is_descriptor<T>>;

/// Helper variable template to detect that a given type is a descriptor.
template <typename T>
inline constexpr bool is_descriptor_v = is_descriptor<T>::value;  // NOLINT

/// A metafunction that returns a mixin type from descriptors.
/**
 * \tparam Base A base mixin template taking descriptors.
 * \tparam Descriptors The descriptors from which to instantiate the mixin.
 */
template <template <class...> class Base, class... Descriptors>
struct mixin_from_descriptors {
  /// The type of the mixin.
  using type = Base<std::decay_t<Descriptors>...>;

  static_assert((is_descriptor_v<Descriptors> && ...), "Invalid mixin descriptors");
};

/// A template specialization that returns a mixin type from a list of descriptors.
/**
 * \tparam Base A base mixin template taking descriptors.
 * \tparam List Any variadic template type.
 * \tparam Descriptors The descriptors from which to instantiate the mixin.
 */
template <template <class...> class Base, template <class...> class List, class... Descriptors>
struct mixin_from_descriptors<Base, List<Descriptors...>> {
  /// The type of the mixin.
  using type = Base<std::decay_t<Descriptors>...>;

  static_assert((is_descriptor_v<Descriptors> && ...), "Invalid mixin descriptors");
};

/// Helper type alias that returns a mixin type from descriptors.
template <template <class...> class Base, class... Descriptors>
using mixin_from_descriptors_t = typename mixin_from_descriptors<Base, Descriptors...>::type;

}  // namespace beluga::mixin

#endif

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

#ifndef BELUGA_PRIMITIVES_HPP
#define BELUGA_PRIMITIVES_HPP

#include <tuple>
#include <type_traits>

#include <beluga/type_traits/strongly_typed_numeric.hpp>
#include <beluga/type_traits/tuple_traits.hpp>

/**
 * \file
 * \brief Implementation of library primitives to abstract member access.
 */

namespace beluga {

/**
 * \page ParticlePage Beluga named requirements: Particle
 * What an implementation of a particle in Beluga should provide.
 *
 * \section ParticleRequirements Requirements
 * `P` is a `Particle` type if given:
 * - An instance `p` of `P`.
 * - A possibly const instance `cp` of `P`.
 *
 * The following is satisfied:
 * - `beluga::state(cp)` is a valid expression and returns the particle state.
 * - `beluga::weight(cp)` is a valid expression and returns the particle weight.
 * - `beluga::state(p)` is a valid expression and returns a mutable reference to the particle state.
 * - `beluga::weight(p)` is a valid expression and returns a mutable reference to the particle weight.
 *
 * \section ParticleNotes Notes
 * Both `beluga::state` and `beluga::weight` are customization point objects. Users have two customization
 * alternatives:
 * - Specifying `state()` and `weight()` as instance members of their custom type.
 * - Creating free functions `state(p)` and `weight(p)` overloaded for their custom type in the same
 *   namespace where that type is defined (so it can be found by ADL).
 */

/// Weight type, as a strongly typed `double`.
using Weight = Numeric<double, struct WeightTag>;

/// Cluster type, as a strongly typed `std::size_t`.
using Cluster = Numeric<std::size_t, struct ClusterTag>;

namespace state_detail {

/// \cond state_detail

template <class T, class = void>
struct has_member_state : std::false_type {};

template <class T>
struct has_member_state<T, std::void_t<decltype(std::declval<T>().state())>> : std::true_type {};

template <typename T>
inline constexpr bool has_member_state_v = has_member_state<T>::value;

/// \endcond

/// Default state implementation.
/**
 * Assumes that the particle is a tuple and the first element is the state.
 */
template <typename T>
constexpr decltype(auto) state(T&& t) noexcept(noexcept(std::get<0>(std::forward<T>(t)))) {
  return std::get<0>(std::forward<T>(t));
}

/// Customization point object type for accessing the `state` of a particle.
/**
 * See https://en.cppreference.com/w/cpp/ranges/cpo.
 */
struct state_fn {
  /// Overload for when the particle type defines a member method.
  template <typename T, std::enable_if_t<has_member_state_v<T>, int> = 0>
  constexpr decltype(auto) operator()(T&& t) const noexcept(noexcept(std::forward<T>(t).state())) {
    return std::forward<T>(t).state();
  }

  /// Overload for when there is an external function that takes this particle type.
  /**
   * The non-member function must be in T's namespace so it can be found by ADL.
   */
  template <typename T, std::enable_if_t<!has_member_state_v<T>, int> = 0>
  constexpr decltype(auto) operator()(T&& t) const noexcept(noexcept(state(std::forward<T>(t)))) {
    return state(std::forward<T>(t));
  }
};

}  // namespace state_detail

/// Customization point object for accessing the `state` of a particle.
inline constexpr state_detail::state_fn state;

namespace weight_detail {

/// \cond weight_detail

template <class T, class = void>
struct has_member_weight : std::false_type {};

template <class T>
struct has_member_weight<T, std::void_t<decltype(std::declval<T>().weight())>> : std::true_type {};

template <typename T>
inline constexpr bool has_member_weight_v = has_member_weight<T>::value;

/// \endcond

/// Default weight implementation.
/**
 * Assumes that the particle is a tuple that has exactly one element of type `beluga::Weight`.
 */
template <typename T>
constexpr decltype(auto) weight(T&& t) noexcept(noexcept(element_of_type<beluga::Weight>(std::forward<T>(t)))) {
  return element_of_type<beluga::Weight>(std::forward<T>(t));
}

/// Customization point object type for accessing the `weight` of a particle.
/**
 * See https://en.cppreference.com/w/cpp/ranges/cpo.
 */
struct weight_fn {
  /// Overload for when the particle type defines a member method.
  template <typename T, std::enable_if_t<has_member_weight_v<T>, int> = 0>
  constexpr decltype(auto) operator()(T&& t) const noexcept(noexcept(std::forward<T>(t).weight())) {
    return std::forward<T>(t).weight();
  }

  /// Overload for when there is an external function that takes this particle type.
  /**
   * The non-member function must be in T's namespace so it can be found by ADL.
   */
  template <typename T, std::enable_if_t<!has_member_weight_v<T>, int> = 0>
  constexpr decltype(auto) operator()(T&& t) const noexcept(noexcept(weight(std::forward<T>(t)))) {
    return weight(std::forward<T>(t));
  }
};

}  // namespace weight_detail

/// Customization point object for accessing the `weight` of a particle.
inline constexpr weight_detail::weight_fn weight;

namespace cluster_detail {

/// \cond cluster_detail

template <class T, class = void>
struct has_member_cluster : std::false_type {};

template <class T>
struct has_member_cluster<T, std::void_t<decltype(std::declval<T>().cluster())>> : std::true_type {};

template <typename T>
inline constexpr bool has_member_cluster_v = has_member_cluster<T>::value;

/// \endcond

/// Default cluster implementation.
/**
 * Assumes that the particle is a tuple that has exactly one element of type `beluga::Cluster`.
 */
template <typename T>
constexpr decltype(auto) cluster(T&& t) noexcept(noexcept(element_of_type<beluga::Cluster>(std::forward<T>(t)))) {
  return element_of_type<beluga::Cluster>(std::forward<T>(t));
}

/// Customization point object type for accessing the `cluster` of a particle.
/**
 * See https://en.cppreference.com/w/cpp/ranges/cpo.
 */
struct cluster_fn {
  /// Overload for when the particle type defines a member method.
  template <typename T, std::enable_if_t<has_member_cluster_v<T>, int> = 0>
  constexpr decltype(auto) operator()(T&& t) const noexcept(noexcept(std::forward<T>(t).cluster())) {
    return std::forward<T>(t).cluster();
  }

  /// Overload for when there is an external function that takes this particle type.
  /**
   * The non-member function must be in T's namespace so it can be found by ADL.
   */
  template <typename T, std::enable_if_t<!has_member_cluster_v<T>, int> = 0>
  constexpr decltype(auto) operator()(T&& t) const noexcept(noexcept(cluster(std::forward<T>(t)))) {
    return cluster(std::forward<T>(t));
  }
};

}  // namespace cluster_detail

/// Customization point object for accessing the `cluster` of a particle.
inline constexpr cluster_detail::cluster_fn cluster;

}  // namespace beluga

#endif

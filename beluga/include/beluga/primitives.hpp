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
#include <concepts/concepts.hpp>

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

/// Definition for the `has_member_state` requirement.
template <typename T>
CPP_requires(has_member_state_, requires(T&& t)(((T&&)t).state()));

/// Definition for the `has_member_state` concept.
template <typename T>
CPP_concept has_member_state = CPP_requires_ref(state_detail::has_member_state_, T);

/// Definition for the `has_non_member_state` requirement.
template <typename T>
CPP_requires(has_non_member_state_, requires(T&& t)(state((T&&)t)));

/// Definition for the `has_non_member_state` concept.
template <typename T>
CPP_concept has_non_member_state = CPP_requires_ref(state_detail::has_non_member_state_, T);

/// Customization point object type for accessing the `state` of a particle.
/**
 * See https://en.cppreference.com/w/cpp/ranges/cpo.
 */
struct state_fn {
  /// Overload for when the particle type defines a member method.
  CPP_template(typename T)(requires(has_member_state<T>))  //
      constexpr decltype(auto)
      operator()(T&& t) const noexcept(noexcept(std::forward<T>(t).state())) {
    return std::forward<T>(t).state();
  }

  /// Overload for when there is an external function that takes this particle type.
  /**
   * The non-member function must be in T's namespace to be found by ADL.
   */
  CPP_template(typename T)(requires(!has_member_state<T> && has_non_member_state<T>))  //
      constexpr decltype(auto)
      operator()(T&& t) const noexcept(noexcept(state(std::forward<T>(t)))) {
    return state(std::forward<T>(t));
  }

  /// Default overload assuming the particle is a tuple.
  CPP_template(typename T)(requires(!has_member_state<T> && !has_non_member_state<T>))  //
      constexpr decltype(auto)
      operator()(T&& t) const noexcept(noexcept(std::get<0>(std::forward<T>(t)))) {
    return std::get<0>(std::forward<T>(t));
  }
};

}  // namespace state_detail

/// Customization point object for accessing the `state` of a particle.
inline constexpr state_detail::state_fn state;

namespace weight_detail {

/// Definition for the `has_member_weight` requirement.
template <typename T>
CPP_requires(has_member_weight_, requires(T&& t)(((T&&)t).weight()));

/// Definition for the `has_member_weight` concept.
template <typename T>
CPP_concept has_member_weight = CPP_requires_ref(weight_detail::has_member_weight_, T);

/// Definition for the `has_non_member_weight` requirement.
template <typename T>
CPP_requires(has_non_member_weight_, requires(T&& t)(weight((T&&)t)));

/// Definition for the `has_non_member_weight` concept.
template <typename T>
CPP_concept has_non_member_weight = CPP_requires_ref(weight_detail::has_non_member_weight_, T);

/// Customization point object type for accessing the `weight` of a particle.
/**
 * See https://en.cppreference.com/w/cpp/ranges/cpo.
 */
struct weight_fn {
  /// Overload for when the particle type defines a member method.
  CPP_template(typename T)(requires(has_member_weight<T>))  //
      constexpr decltype(auto)
      operator()(T&& t) const noexcept(noexcept(std::forward<T>(t).weight())) {
    return std::forward<T>(t).weight();
  }

  /// Overload for when there is an external function that takes this particle type.
  /**
   * The non-member function must be in T's namespace to be found by ADL.
   */
  CPP_template(typename T)(requires(!has_member_weight<T> && has_non_member_weight<T>))  //
      constexpr decltype(auto)
      operator()(T&& t) const noexcept(noexcept(weight(std::forward<T>(t)))) {
    return weight(std::forward<T>(t));
  }

  /// Default overload assuming the particle is a tuple.
  CPP_template(typename T)(requires(!has_member_weight<T> && !has_non_member_weight<T>))  //
      constexpr decltype(auto)
      operator()(T&& t) const noexcept(noexcept(element_of_type<beluga::Weight>(std::forward<T>(t)))) {
    return element_of_type<beluga::Weight>(std::forward<T>(t));
  }
};

}  // namespace weight_detail

/// Customization point object for accessing the `weight` of a particle.
inline constexpr weight_detail::weight_fn weight;

namespace cluster_detail {

/// Definition for the `has_member_cluster` requirement.
template <typename T>
CPP_requires(has_member_cluster_, requires(T&& t)(((T&&)t).cluster()));

/// Definition for the `has_member_cluster` concept.
template <typename T>
CPP_concept has_member_cluster = CPP_requires_ref(cluster_detail::has_member_cluster_, T);

/// Definition for the `has_non_member_cluster` requirement.
template <typename T>
CPP_requires(has_non_member_cluster_, requires(T&& t)(cluster(std::forward<T>(t))));

/// Definition for the `has_non_member_cluster` concept.
template <typename T>
CPP_concept has_non_member_cluster = CPP_requires_ref(cluster_detail::has_non_member_cluster_, T);

/// Customization point object type for accessing the `cluster` of a particle.
/**
 * See https://en.cppreference.com/w/cpp/ranges/cpo.
 */
struct cluster_fn {
  /// Overload for when the particle type defines a member method.
  CPP_template(typename T)(requires(has_member_cluster<T>))  //
      constexpr decltype(auto)
      operator()(T&& t) const noexcept(noexcept(std::forward<T>(t).cluster())) {
    return std::forward<T>(t).cluster();
  }

  /// Overload for when there is an external function that takes this particle type.
  /**
   * The non-member function must be in T's namespace to be found by ADL.
   */
  CPP_template(typename T)(requires(!has_member_cluster<T> && has_non_member_cluster<T>))  //
      constexpr decltype(auto)
      operator()(T&& t) const noexcept(noexcept(cluster(std::forward<T>(t)))) {
    return cluster(std::forward<T>(t));
  }

  /// Default overload assuming the particle is a tuple.
  CPP_template(typename T)(requires(!has_member_cluster<T> && !has_non_member_cluster<T>))  //
      constexpr decltype(auto)
      operator()(T&& t) const noexcept(noexcept(element_of_type<beluga::Cluster>(std::forward<T>(t)))) {
    return element_of_type<beluga::Cluster>(std::forward<T>(t));
  }
};

}  // namespace cluster_detail

/// Customization point object for accessing the `cluster` of a particle.
inline constexpr cluster_detail::cluster_fn cluster;

}  // namespace beluga

#endif

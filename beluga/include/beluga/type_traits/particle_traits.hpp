// Copyright 2022-2024 Ekumen, Inc.
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

#ifndef BELUGA_TYPE_TRAITS_PARTICLE_TRAITS_HPP
#define BELUGA_TYPE_TRAITS_PARTICLE_TRAITS_HPP

#include <beluga/primitives.hpp>
#include <beluga/type_traits/tuple_traits.hpp>
#include <range/v3/range/traits.hpp>

/**
 * \file
 * \brief Implementation of traits for particle types, see the \ref ParticlePage "Particle" named requirements.
 */

namespace beluga {

/// Common traits of all particle types. See \ref ParticlePage "Page" requirements as well.
template <class T>
struct particle_traits {
  /// The particle state type.
  using state_type = std::decay_t<decltype(beluga::state(std::declval<T>()))>;
  /// The particle weight type.
  using weight_type = std::decay_t<decltype(beluga::weight(std::declval<T>()))>;
};

/// Type trait that returns the state type given a particle type.
template <class T>
using state_t = typename particle_traits<T>::state_type;

/// Type trait that returns the weight type given a particle type.
template <class T>
using weight_t = typename particle_traits<T>::weight_type;

/// \cond

template <class T, class = void>
struct has_state : public std::false_type {};

template <class T>
struct has_state<T, std::void_t<decltype(beluga::state(std::declval<T>()))>> : std::true_type {};

template <class T>
inline constexpr bool has_state_v = has_state<T>::value;

template <class T, class = void>
struct has_weight : public std::false_type {};

template <class T>
struct has_weight<T, std::void_t<decltype(beluga::weight(std::declval<T>()))>> : std::true_type {};

template <class T>
inline constexpr bool has_weight_v = has_weight<T>::value;

template <class T, class = void>
struct is_particle : public std::false_type {};

template <class T>
struct is_particle<T, std::enable_if_t<std::conjunction_v<has_state<T>, has_weight<T>>>> : std::true_type {};

template <class T>
inline constexpr bool is_particle_v = is_particle<T>::value;

template <class R>
inline constexpr bool is_particle_range_v = is_particle_v<ranges::range_value_t<R>>;

/// \endcond

namespace detail {

/// Function object type to create a particle from a given state.
template <class Particle, class State = state_t<Particle>>
struct make_from_state_fn {
  /// Returns a new particle from the given state.
  /**
   * \param value The state to make the particle from.
   * \return The new particle, created from the given state.
   *
   * The new particle will have a weight equal to 1.
   */
  constexpr auto operator()(State value) const {
    static_assert(is_particle_v<Particle>);
    auto particle = []() {
      if constexpr (is_tuple_like_v<Particle>) {
        // Support for zipped ranges composed with views that don't
        // propagate the tuple value type of the original range
        // (ranges::views::const_).
        return decay_tuple_like_t<Particle>{};
      } else {
        return Particle{};
      }
    }();
    beluga::state(particle) = std::move(value);
    beluga::weight(particle) = 1.0;
    return particle;
  }
};

}  // namespace detail

/// A function object to create a particle from a given state.
/**
 * Takes a state and returns a new particle with that state.
 * The new particle is given a weight of 1.
 *
 * \tparam Particle The particle type to be created.
 */
template <class Particle>
inline constexpr detail::make_from_state_fn<Particle> make_from_state;

}  // namespace beluga

#endif

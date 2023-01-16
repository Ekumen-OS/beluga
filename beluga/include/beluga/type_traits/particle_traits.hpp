// Copyright 2022 Ekumen, Inc.
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

#include <tuple>
#include <type_traits>

#include <beluga/type_traits/container_traits.hpp>
#include <beluga/views.hpp>

/**
 * \file
 * \brief Implementation of traits for particle types, see the \ref ParticlePage "Particle" named requirements.
 */

namespace beluga {

/// std::decay is applied to  all tuple element types if T is a tuple.
template <class T>
struct decay_tuple_types {
  /// Decayed type.
  using type = T;
};

/// Specialization for tuples.
template <template <class...> class Tuple, class... Types>
struct decay_tuple_types<Tuple<Types...>> {
  using type = Tuple<std::decay_t<Types>...>;
};

/// Same as `decay_tuple_types<T>::type`.
template <class T>
using decay_tuple_types_t = typename decay_tuple_types<T>::type;

/// std::decay is applied to T, and to all tuple element types if T is a tuple.
template <class T>
struct decay_tuple {
  using type = decay_tuple_types_t<std::decay_t<T>>;
};

/// Same as `decay_tuple<T>::type`.
template <class T>
using decay_tuple_t = typename decay_tuple<T>::type;

/// Common traits of all particle types. See \ref ParticlePage "Page" requirements as well.
template <class T>
struct particle_traits {};

/// Specialization for particles represented by a state/weight/etc tuple.
template <template <class...> class Pair, class State, class... Extra>
struct particle_traits<Pair<State, double, Extra...>> {
  /// The particle state type.
  using state_type = State;
  /// The particle weight type.
  using weight_type = double;
  /// The particle state/weight pair.
  using value_type = Pair<State, double>;

  /// Returns the particle state.
  template <class T>
  static constexpr decltype(auto) state(T&& particle) {
    return std::get<0>(std::forward<T>(particle));
  }

  /// Returns the particle weight.
  template <class T>
  static constexpr decltype(auto) weight(T&& particle) {
    return std::get<1>(std::forward<T>(particle));
  }

  /// Returns a [range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
  /// of the particle state.
  static constexpr auto states_view() { return views::elements<0>; }

  /// Returns a [range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
  /// of the particle weight.
  static constexpr auto weights_view() { return views::elements<1>; }
};

/// Specialization for particles represented by a state/weight/cluster/etc tuple.
template <template <class...> class Tuple, class State, class... Extra>
struct particle_traits<Tuple<State, double, std::size_t, Extra...>> {
  /// The particle state type.
  using state_type = State;
  /// The particle weight type.
  using weight_type = double;
  /// The particle state/weight/cluster tuple.
  using value_type = Tuple<State, double, std::size_t>;

  /// Returns the particle state.
  template <class T>
  static constexpr decltype(auto) state(T&& particle) {
    return std::get<0>(std::forward<T>(particle));
  }

  /// Returns the particle weight.
  template <class T>
  static constexpr decltype(auto) weight(T&& particle) {
    return std::get<1>(std::forward<T>(particle));
  }

  /// Returns the particle cluster.
  template <class T>
  static constexpr decltype(auto) cluster(T&& particle) {
    return std::get<2>(std::forward<T>(particle));
  }

  /// Returns a [range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
  /// of the particle state.
  static constexpr auto states_view() { return views::elements<0>; }

  /// Returns a [range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
  /// of the particle weight.
  static constexpr auto weights_view() { return views::elements<1>; }

  /// Returns a [range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
  /// of the particle cluster.
  static constexpr auto clusters_view() { return views::elements<2>; }
};

/// Gets the state of a particle.
/**
 * \tparam T The particle type.
 * \param particle The particle to get the state from.
 * \return The state of the particle.
 */
template <class T>
constexpr decltype(auto) state(T&& particle) {
  return particle_traits<decay_tuple_t<T>>::state(std::forward<T>(particle));
}

/// Gets the weight of a particle.
/**
 * \tparam T The particle type.
 * \param particle The particle to get the weight from.
 * \return The weight of the particle.
 */
template <class T>
constexpr decltype(auto) weight(T&& particle) {
  return particle_traits<decay_tuple_t<T>>::weight(std::forward<T>(particle));
}

/// Gets the cluster of a particle.
/**
 * \tparam T The particle type.
 * \param particle The particle to get the cluster from.
 * \return The cluster of the particle.
 */
template <class T>
constexpr decltype(auto) cluster(T&& particle) {
  return particle_traits<decay_tuple_t<T>>::cluster(std::forward<T>(particle));
}

namespace views {

/// Returns a [range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
/// of the particle state.
/**
 * \tparam T The particle type.
 * \return A range adaptor for the particle state.
 */
template <class T>
constexpr auto states() {
  return particle_traits<decay_tuple_t<T>>::states_view();
}

/// Returns a [range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
/// of the particle weight.
/**
 * \tparam T The particle type.
 * \return A range adaptor for the particle weight.
 */
template <class T>
constexpr auto weights() {
  return particle_traits<decay_tuple_t<T>>::weights_view();
}

/// Returns a [range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
/// of the particle cluster.
/**
 * \tparam T The particle type.
 * \return A range adaptor for the particle cluster.
 */
template <class T>
constexpr auto clusters() {
  return particle_traits<decay_tuple_t<T>>::clusters_view();
}

/// Returns a view of the particles states in the container.
/**
 * \tparam Container A container of particles.
 * \param container The container of particles to be adapted.
 * \return The view of the particles states.
 */
template <class Container>
constexpr auto states(Container&& container) {
  using T = typename std::decay_t<Container>::value_type;
  return all(std::forward<Container>(container)) | states<T>();
}

/// Returns a view of the particles weights in the container.
/**
 * \tparam Container A container of particles.
 * \param container The container of particles to be adapted.
 * \return The view of the particles weights.
 */
template <class Container>
constexpr auto weights(Container&& container) {
  using T = typename std::decay_t<Container>::value_type;
  return all(std::forward<Container>(container)) | weights<T>();
}

/// Returns a view of the particles clusters in the container.
/**
 * \tparam Container A container of particles.
 * \param container The container of particles to be adapted.
 * \return The view of the particles clusters.
 */
template <class Container>
constexpr auto clusters(Container&& container) {
  using T = typename std::decay_t<Container>::value_type;
  return all(std::forward<Container>(container)) | clusters<T>();
}

}  // namespace views

/// Returns a new particle from the give state.
/**
 * \tparam Particle The particle type to be used.
 * \tparam T The particle state type.
 *  T must be convertible to `particle_traits<Particle>::state_type`.
 * \param value The state to make the particle from.
 * \return The new particle, created from the given state.
 */
template <class Particle, class T = typename particle_traits<Particle>::state_type>
constexpr auto make_from_state(T&& value) {
  auto particle = Particle{};
  state(particle) = std::forward<T>(value);
  return particle;
}

}  // namespace beluga

#endif

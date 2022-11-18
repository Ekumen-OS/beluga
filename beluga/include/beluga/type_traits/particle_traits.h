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

#pragma once

#include <tuple>
#include <type_traits>

#include <beluga/type_traits/container_traits.h>
#include <beluga/views.h>

namespace beluga {

template <class T>
struct decay_tuple_types {
  using type = T;
};

template <template <class...> class Tuple, class... Types>
struct decay_tuple_types<Tuple<Types...>> {
  using type = Tuple<std::decay_t<Types>...>;
};

template <class T>
using decay_tuple_types_t = typename decay_tuple_types<T>::type;

template <class T>
struct decay_tuple {
  using type = decay_tuple_types_t<std::decay_t<T>>;
};

template <class T>
using decay_tuple_t = typename decay_tuple<T>::type;

template <class T>
struct particle_traits {};

template <template <class...> class Pair, class State, class... Extra>
struct particle_traits<Pair<State, double, Extra...>> {
  using state_type = State;
  using value_type = Pair<State, double>;

  template <class T>
  static constexpr decltype(auto) state(T&& particle) {
    return std::get<0>(std::forward<T>(particle));
  }

  template <class T>
  static constexpr decltype(auto) weight(T&& particle) {
    return std::get<1>(std::forward<T>(particle));
  }

  static constexpr auto states_view() { return views::elements<0>; }

  static constexpr auto weights_view() { return views::elements<1>; }
};

template <template <class...> class Tuple, class State, class... Extra>
struct particle_traits<Tuple<State, double, std::size_t, Extra...>> {
  using state_type = State;
  using value_type = Tuple<State, double, std::size_t>;

  template <class T>
  static constexpr decltype(auto) state(T&& particle) {
    return std::get<0>(std::forward<T>(particle));
  }

  template <class T>
  static constexpr decltype(auto) weight(T&& particle) {
    return std::get<1>(std::forward<T>(particle));
  }

  template <class T>
  static constexpr decltype(auto) cluster(T&& particle) {
    return std::get<2>(std::forward<T>(particle));
  }

  static constexpr auto states_view() { return views::elements<0>; }

  static constexpr auto weights_view() { return views::elements<1>; }

  static constexpr auto clusters_view() { return views::elements<2>; }
};

template <class T>
constexpr decltype(auto) state(T&& particle) {
  return particle_traits<decay_tuple_t<T>>::state(std::forward<T>(particle));
}

template <class T>
constexpr decltype(auto) weight(T&& particle) {
  return particle_traits<decay_tuple_t<T>>::weight(std::forward<T>(particle));
}

template <class T>
constexpr decltype(auto) cluster(T&& particle) {
  return particle_traits<decay_tuple_t<T>>::cluster(std::forward<T>(particle));
}

namespace views {

template <class T>
constexpr auto states() {
  return particle_traits<decay_tuple_t<T>>::states_view();
}

template <class T>
constexpr auto weights() {
  return particle_traits<decay_tuple_t<T>>::weights_view();
}

template <class T>
constexpr auto clusters() {
  return particle_traits<decay_tuple_t<T>>::clusters_view();
}

template <class Container>
constexpr auto states(Container&& container) {
  using T = typename std::decay_t<Container>::value_type;
  return all(std::forward<Container>(container)) | states<T>();
}

template <class Container>
constexpr auto weights(Container&& container) {
  using T = typename std::decay_t<Container>::value_type;
  return all(std::forward<Container>(container)) | weights<T>();
}

template <class Container>
constexpr auto clusters(Container&& container) {
  using T = typename std::decay_t<Container>::value_type;
  return all(std::forward<Container>(container)) | clusters<T>();
}

}  // namespace views

template <class Particle, class T = typename particle_traits<Particle>::state_type>
constexpr auto make_from_state(T&& value) {
  auto particle = Particle{};
  state(particle) = std::forward<T>(value);
  return particle;
}

}  // namespace beluga

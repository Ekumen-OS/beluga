#pragma once

#include <tuple>
#include <type_traits>

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

}  // namespace views

}  // namespace beluga

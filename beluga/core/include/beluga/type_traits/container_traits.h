#pragma once

#include <type_traits>

#include <range/v3/view/all.hpp>

namespace beluga {

template <class T>
struct container_traits {
  using value_type = typename T::value_type;
  using size_type = typename T::size_type;

  template <class U>
  static constexpr auto view_all(U&& container) {
    return container | ranges::views::all;
  }
};

namespace views {

template <class T>
constexpr auto all(T&& container) {
  return container_traits<std::decay_t<T>>::view_all(container);
}

}  // namespace views

}  // namespace beluga

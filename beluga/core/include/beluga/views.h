#pragma once

#include <tuple>

#include <range/v3/view.hpp>

namespace beluga::views {

template <std::size_t N>
inline auto elements = ranges::views::transform([](auto&& particle) -> decltype(auto) {
  return std::get<N>(std::forward<decltype(particle)>(particle));
});

template <class Container>
inline auto all(Container& container) {
  return container | ranges::views::all;
}

}  // namespace beluga::views

#pragma once

#include <tuple>

#include <range/v3/view/transform.hpp>

namespace beluga::views {

template <std::size_t N>
inline auto elements = ranges::views::transform([](auto&& particle) -> decltype(auto) {
  return std::get<N>(std::forward<decltype(particle)>(particle));
});

}  // namespace beluga::views

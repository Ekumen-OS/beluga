#pragma once

#include <bitset>
#include <cmath>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>

namespace beluga {

template <class T, typename Enable = void>
struct voxel_hash {};

template <template <class...> class Tuple, class... Types>
struct voxel_hash<
    Tuple<Types...>,
    std::enable_if_t<(std::is_arithmetic_v<Types> && ...) && sizeof...(Types) <= 3, void>> {
 public:
  constexpr std::size_t operator()(const Tuple<Types...>& tuple, double voxel_size = 1.) const {
    return hash_impl(tuple, voxel_size, std::make_index_sequence<sizeof...(Types)>());
  }

 private:
  template <std::size_t... Ids>
  static constexpr std::size_t hash_impl(const Tuple<Types...>& tuple, double voxel_size, std::index_sequence<Ids...>) {
    constexpr auto kBits = std::numeric_limits<std::size_t>::digits / sizeof...(Ids);
    return (round_and_shift<kBits, Ids>(std::get<Ids>(tuple) / voxel_size) | ...);
  }

  template <std::size_t N, std::size_t I, class T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
  static constexpr std::size_t round_and_shift(T value) {
    return std::bitset<N>(std::llround(value)).to_ullong() << (I * N);
  }
};

}  // namespace beluga

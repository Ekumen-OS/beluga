#pragma once

#include <bitset>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>

namespace beluga {

template <class T, typename Enable = void>
struct voxel_hash {};

template <template <class...> class Tuple, class... Types>
struct voxel_hash<Tuple<Types...>, std::enable_if_t<(std::is_arithmetic_v<Types> && ...), void>> {
 public:
  constexpr std::size_t operator()(const Tuple<Types...>& tuple, double voxel_size = 1.) const {
    return hash_impl(tuple, voxel_size, std::make_index_sequence<sizeof...(Types)>());
  }

 private:
  template <std::size_t... Ids>
  static constexpr std::size_t hash_impl(const Tuple<Types...>& tuple, double voxel_size, std::index_sequence<Ids...>) {
    constexpr auto kBits = std::numeric_limits<std::size_t>::digits / sizeof...(Ids);
    return (floor_and_shift<kBits, Ids>(std::get<Ids>(tuple) / voxel_size) | ...);
  }

  template <std::size_t N, std::size_t I>
  static constexpr std::size_t floor_and_shift(double value) {
    // Compute the largest integer value not greater than value.
    auto signed_value = static_cast<std::intmax_t>(std::floor(value));
    // Compute the smallest unsigned integer equal to the source value modulo 2^n
    // (where n is the number of bits used to represent the destination type).
    auto unsigned_value = static_cast<std::uintmax_t>(signed_value);
    // Create a fixed-size sequence of N bits and perform a binary shift left I * N positions.
    return std::bitset<N>{unsigned_value}.to_ullong() << (I * N);
  }
};

}  // namespace beluga

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

#include <bitset>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>

#include <sophus/se2.hpp>

namespace beluga {

namespace detail {

template <std::size_t N, std::size_t I>
constexpr std::size_t floor_and_shift(double value) {
  // Compute the largest integer value not greater than value.
  auto signed_value = static_cast<std::intmax_t>(std::floor(value));
  // Compute the smallest unsigned integer equal to the source value modulo 2^n
  // (where n is the number of bits used to represent the destination type).
  auto unsigned_value = static_cast<std::uintmax_t>(signed_value);
  // Create a fixed-size sequence of N bits and perform a binary shift left I * N positions.
  return std::bitset<N>{unsigned_value}.to_ullong() << (I * N);
}

template <class T, std::size_t... Ids>
constexpr std::size_t hash_impl(const T& value, double resolution, std::index_sequence<Ids...>) {
  constexpr auto kBits = std::numeric_limits<std::size_t>::digits / sizeof...(Ids);
  return (detail::floor_and_shift<kBits, Ids>(std::get<Ids>(value) / resolution) | ...);
}

}  // namespace detail

template <class T, typename Enable = void>
struct spatial_hash {};

template <class T, std::size_t N>
struct spatial_hash<std::array<T, N>, std::enable_if_t<std::is_arithmetic_v<T>, void>> {
 public:
  constexpr std::size_t operator()(const std::array<T, N>& array, double resolution = 1.) const {
    return detail::hash_impl(array, resolution, std::make_index_sequence<N>());
  }
};

template <template <class...> class Tuple, class... Types>
struct spatial_hash<Tuple<Types...>, std::enable_if_t<(std::is_arithmetic_v<Types> && ...), void>> {
 public:
  constexpr std::size_t operator()(const Tuple<Types...>& tuple, double resolution = 1.) const {
    return detail::hash_impl(tuple, resolution, std::make_index_sequence<sizeof...(Types)>());
  }
};

template <>
struct spatial_hash<Sophus::SE2d, void> {
 public:
  std::size_t operator()(const Sophus::SE2d& state, double resolution = 1.) const {
    const auto& position = state.translation();
    return spatial_hash<std::tuple<double, double>>{}(std::make_tuple(position.x(), position.y()), resolution);
  }
};

}  // namespace beluga

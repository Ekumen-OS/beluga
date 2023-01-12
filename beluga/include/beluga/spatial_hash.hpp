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

#ifndef BELUGA_SPATIAL_HASH_HPP
#define BELUGA_SPATIAL_HASH_HPP

#include <bitset>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>

#include <sophus/se2.hpp>

/**
 * \file Implementation of a spatial hash for N dimensional states.
 */
// TODO(ivanpauno): Move to algorithm?

namespace beluga {

namespace detail {

/// Returns the floor of a value and shift it.
/**
 * \tparam N Number of bits to be used in the result, the least significant will be used.
 * \tparam I Result will be shifted by I*N.
 * \param value Input value to be floored and shifted.
 * \return The calculated result.
 */
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

/// Hashes a tuple or array of scalar types, using a resolution for each element and using the same
/// amount of bits for them.
/**
 * \tparam T Tuple or array of scalar types.
 * \tparam ...Ids Indexes of the array/tuple to be used to calculate the hash.
 * \param value The array/tuple to be hashed.
 * \param resolution The resolution to be used.
 * \param  Index sequence, only to allow unpacking ...Ids.
 * \return The calculated hash.
 */
template <class T, std::size_t... Ids>
constexpr std::size_t hash_impl(const T& value, double resolution, std::index_sequence<Ids...>) {
  constexpr auto kBits = std::numeric_limits<std::size_t>::digits / sizeof...(Ids);
  return (detail::floor_and_shift<kBits, Ids>(std::get<Ids>(value) / resolution) | ...);
}

}  // namespace detail

/// Callable class, allowing to calculate the hash of a particle state.
template <class T, typename Enable = void>
struct spatial_hash {};

/// Specialization for arrays.
template <class T, std::size_t N>
struct spatial_hash<std::array<T, N>, std::enable_if_t<std::is_arithmetic_v<T>, void>> {
 public:
  /// Calculates the array hash, using the given resolution in all axes.
  /**
   * \param array Array to be hashed.
   * \param resolution Resolution to be used in each axes.
   * \return The calculated hash.
   */
  constexpr std::size_t operator()(const std::array<T, N>& array, double resolution = 1.) const {
    return detail::hash_impl(array, resolution, std::make_index_sequence<N>());
  }
};

/// Specialization for tuples.
template <template <class...> class Tuple, class... Types>
struct spatial_hash<Tuple<Types...>, std::enable_if_t<(std::is_arithmetic_v<Types> && ...), void>> {
 public:
  /// Calculates the tuple hash, using the given resolution in all axes.
  /**
   * \param tuple Tuple to be hashed.
   * \param resolution Resolution to be used in each axes.
   * \return The calculated hash.
   */
  constexpr std::size_t operator()(const Tuple<Types...>& tuple, double resolution = 1.) const {
    return detail::hash_impl(tuple, resolution, std::make_index_sequence<sizeof...(Types)>());
  }
};

// TODO(ivanpauno): One may want to use a different spatial_hash for the same type, e.g. for SE2d
// one may also want to consider the rotation.
// Instead of using spatial_hash<> directly in KldResampling and specialization here, the spatial hash to be used
// could be specified as a template parameter.
/// Specialization for Sophus::SE2d.
/**
 * Will calculate the spatial hash based on the translation, the rotation is not used.
 */
template <>
struct spatial_hash<Sophus::SE2d, void> {
 public:
  /// Calculates the tuple hash, using the given resolution for both the x and y translation coordinates.
  /**
   * \param state The state to be hashed.
   * \param resolution Resolution to be used for both x and y translation coordinates.
   * \return The calculated hash.
   */
  std::size_t operator()(const Sophus::SE2d& state, double resolution = 1.) const {
    const auto& position = state.translation();
    return spatial_hash<std::tuple<double, double>>{}(std::make_tuple(position.x(), position.y()), resolution);
  }
};

}  // namespace beluga

#endif

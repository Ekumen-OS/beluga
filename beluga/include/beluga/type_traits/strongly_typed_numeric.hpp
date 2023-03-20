// Copyright 2023 Ekumen, Inc.
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

#ifndef BELUGA_TYPE_TRAITS_STRONGLY_TYPED_NUMERIC_HPP
#define BELUGA_TYPE_TRAITS_STRONGLY_TYPED_NUMERIC_HPP

#include <functional>
#include <limits>
#include <type_traits>
/**
 * \file
 * \brief Implementation of a strongly typed numeric helper.
 */
namespace beluga {

/// Helper for creating strongly typed numeric types.
/** Usage:
 *    using YOUR_TYPE = Numeric< UNDERLYING_TYPE , struct UNDERLYING_TYPETag >;
 *
 * Types resulting from this pattern will allow for implicit conversions in both ways (from underlying type, and to
 * underlying type). This results in a more convenient usage, but not type safety strictly speaking.
 * `std::numeric_limits` and `std::hash` will be specialized for this new type.
 * \tparam T Underlying type, i.e. 'double'.
 * \tparam PhantomType empty type to use as phantom type.
 */
template <typename T, typename PhantomType, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
class Numeric final {
 public:
  /// Default constructor.
  constexpr Numeric() noexcept = default;
  /// Constructs a new Numeric object.
  /**
   *  \param t built-in arithmetic type value.
   */
  constexpr Numeric(const T t) noexcept : value_{t} {};  // NOLINT
  /// Implicit conversion to the underlying type.
  constexpr operator T&() noexcept { return value_; }  // NOLINT
  /// \overload
  constexpr operator const T&() const noexcept { return value_; }  // NOLINT

 private:
  T value_{0};  // Underlying primitive type.
};

}  // namespace beluga
namespace std {

/// `std::numeric_limits` specialization for `Numeric` types.
/**
 * \tparam T Underlying type.
 * \tparam PhantomType Phantom type.
 */
template <typename T, typename PhantomType>
struct numeric_limits<beluga::Numeric<T, PhantomType>> : public numeric_limits<T> {};

/// `std::hash` specialization for `Numeric` types.
/**
 * \tparam T Underlying type.
 * \tparam PhantomType Phantom type.
 */
template <typename T, typename PhantomType>
struct hash<beluga::Numeric<T, PhantomType>> {
  /// Forwards hashing to the underlying type.
  /**
   * \param t Numeric instance to hash.
   * \return size_t Hash.
   */
  size_t operator()(beluga::Numeric<T, PhantomType> t) { return hash<T>{}(t); }
};

}  // namespace std

#endif

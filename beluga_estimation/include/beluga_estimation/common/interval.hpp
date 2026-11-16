// Copyright 2026 Ekumen, Inc.
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

#include <optional>
#include <utility>

namespace beluga_estimation {

/// @brief Represents an interval with optional lower and upper bounds.
///
/// This struct is templated on a value type `T`.
/// Both the lower and upper bounds of the interval are optional, meaning an
/// interval can be open-ended on either side.
///
/// Each bound carries a `closed` flag: a closed bound is part of the
/// interval, an open bound is not.
///
/// @tparam T The type of the values defining the interval.
template <class T>
struct Interval {
  /// @brief A bound of the interval.
  struct Bound {
    /// @brief The value of the bound.
    T value;
    /// @brief Whether the bound is part of the interval (closed) or not (open).
    bool closed;
  };

  /// @brief Factory method to create a closed interval.
  /// @param lower The lower bound of the interval.
  /// @param upper The upper bound of the interval.
  /// @return An interval instance with closed lower and upper bounds
  ///   (i.e., $[lower, upper]$).
  static Interval Closed(T lower, T upper) {
    return Interval{Bound{std::move(lower), true}, Bound{std::move(upper), true}};
  }

  /// @brief Factory method to create an open interval.
  /// @param lower The lower bound of the interval.
  /// @param upper The upper bound of the interval.
  /// @return An interval instance with open lower and upper bounds
  ///   (i.e., $(lower, upper)$).
  static Interval Open(T lower, T upper) {
    return Interval{Bound{std::move(lower), false}, Bound{std::move(upper), false}};
  }

  /// @brief Factory method to create an open interval.
  /// @param lower The lower bound of the interval.
  /// @param upper The upper bound of the interval.
  /// @return An interval instance with open lower and upper bounds
  ///   (i.e., $(lower, upper)$). Alias of @ref Open.
  static Interval Between(T lower, T upper) { return Open(std::move(lower), std::move(upper)); }

  /// @brief Factory method to create a right-open interval.
  /// @param lower The lower bound of the interval.
  /// @param upper The optional upper bound of the interval.
  /// @return An interval instance with a closed lower bound and an open upper
  ///   bound (i.e., $[lower, upper)$). If @p upper is not provided, the
  ///   interval is unbounded above (i.e., $[lower, \infty)$).
  static Interval RightOpen(T lower, std::optional<T> upper = std::nullopt) {
    return Interval{
        Bound{std::move(lower), true}, upper ? std::optional<Bound>{Bound{std::move(*upper), false}} : std::nullopt};
  }

  /// @brief Factory method to create a left-open interval.
  /// @param lower The optional lower bound of the interval.
  /// @param upper The upper bound of the interval.
  /// @return An interval instance with an open lower bound and a closed upper
  ///   bound (i.e., $(lower, upper]$). If @p lower is not provided, the
  ///   interval is unbounded below (i.e., $(-\infty, upper]$).
  static Interval LeftOpen(std::optional<T> lower, T upper) {
    return Interval{
        lower ? std::optional<Bound>{Bound{std::move(*lower), false}} : std::nullopt, Bound{std::move(upper), true}};
  }

  /// @brief Factory method to create an interval that starts at a specified value.
  /// @param start The start value of the interval.
  /// @return An interval instance with a closed lower bound and no upper bound
  ///   (i.e., $[start, \infty)$).
  static Interval From(T start) { return Interval{Bound{std::move(start), true}, std::nullopt}; }

  /// @brief Factory method to create an interval that ends at a specified value.
  /// @param end The end value of the interval.
  /// @return An interval instance with no lower bound and an open upper bound
  ///   (i.e., $(-\infty, end)$).
  static Interval Until(T end) { return Interval{std::nullopt, Bound{std::move(end), false}}; }

  /// @brief Factory method to create an interval that starts after a specified value.
  /// @param start The start value of the interval.
  /// @return An interval instance with an open lower bound and no upper bound
  ///   (i.e., $(start, \infty)$).
  static Interval Since(T start) { return Interval{Bound{std::move(start), false}, std::nullopt}; }

  /// @brief The optional lower bound of the interval.
  std::optional<Bound> lower_bound;
  /// @brief The optional upper bound of the interval.
  std::optional<Bound> upper_bound;
};

}  // namespace beluga_estimation

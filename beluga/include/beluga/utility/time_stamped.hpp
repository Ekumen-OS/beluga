// Copyright 2023-2024 Ekumen, Inc.
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

#ifndef BELUGA_UTILITY_TIME_STAMPED_HPP
#define BELUGA_UTILITY_TIME_STAMPED_HPP

#include <chrono>

/**
 * \file
 * \brief Implementation of a time-stamped wrapper for values.
 */

namespace beluga {

/// A wrapper that associates a value with a timestamp.
/**
 * This structure provides a way to bundle any value with its associated timestamp,
 * which is commonly needed in motion models and sensor processing where timing
 * information is crucial for proper calculations.
 *
 * \tparam T The type of the value to be time-stamped.
 * \tparam ClockT The clock type to use for timestamps. Defaults to system_clock.
 */
template <typename T, typename ClockT = std::chrono::system_clock>
struct TimeStamped {
  /// The wrapped value.
  T value;

  /// The timestamp associated with the value.
  std::chrono::time_point<ClockT> timestamp;
  /// Default constructor.
  /**
   * Initializes value with default construction and timestamp to epoch.
   */
  TimeStamped() : value{}, timestamp{} {}

  /// Constructs a TimeStamped with the current time.
  /**
   * \param val The value to be time-stamped.
   */
  explicit TimeStamped(const T& val) : value(val), timestamp{} {}

  /// Constructs a TimeStamped with a specific timestamp.
  /**
   * \param val The value to be time-stamped.
   * \param ts The timestamp to associate with the value.
   */
  TimeStamped(const T& val, std::chrono::time_point<ClockT> ts) : value(val), timestamp(ts) {}

  /// Implicit conversion operator to the wrapped value.
  /**
   * This allows TimeStamped<T> to be used anywhere a T is expected,
   * enabling motion models that don't need timestamps to work transparently
   * with both TimeStamped<T> and T types.
   */
  operator const T&() const { return value; }

  /// Pointer-like access operator.
  /**
   * This allows TimeStamped<T> to be used with -> syntax for accessing
   * methods of the wrapped value.
   */
  const T* operator->() const { return &value; }
};

}  // namespace beluga

#endif

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

#ifndef BELUGA_TEST_MOTION_UTILS_HPP
#define BELUGA_TEST_MOTION_UTILS_HPP

#include <chrono>
#include <tuple>

#include <beluga/utility/time_stamped.hpp>

/**
 * \file
 * \brief Test utilities for motion models.
 */

namespace beluga::testing {
/// Helper function to create a control action tuple with TimeStamped values and explicit timestamps for testing
template <typename T, typename ClockT = std::chrono::system_clock>
auto make_control_action(
    const T& current,
    const T& previous,
    std::chrono::time_point<ClockT> current_timestamp,
    std::chrono::time_point<ClockT> previous_timestamp) {
  return std::make_tuple(
      TimeStamped<T, ClockT>{current, current_timestamp}, TimeStamped<T, ClockT>{previous, previous_timestamp});
}

}  // namespace beluga::testing

#endif  // BELUGA_TEST_MOTION_UTILS_HPP

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

/**
 * @file
 * @brief Includes clock interface declarations.
 */

#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation {

/**
 * @brief Abstract interface for clock access.
 *
 * Provides a middleware-agnostic way to query the current time,
 * enabling time-based logic to remain decoupled from the underlying
 * middleware (e.g., ROS 2 simulation time vs. wall clock).
 */
class ClockInterface {
 public:
  virtual ~ClockInterface() = default;

  /// Returns the current time according to the underlying clock.
  virtual TimePoint Now() const = 0;
};

}  // namespace beluga_estimation

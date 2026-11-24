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
 * @brief Includes clock interface mock for testing.
 */

#include <chrono>

#include <beluga_estimation/middleware/clock_interface.hpp>

namespace beluga_estimation {

/**
 * @brief Mock implementation of ClockInterface for testing.
 *
 * Provides a controllable clock that can be set to specific times or
 * advanced by durations, enabling deterministic time-based testing.
 */
class MockClock : public ClockInterface {
 public:
  /// Constructs a mock clock starting at time zero.
  MockClock() : current_time_(std::chrono::seconds(0)) {}

  TimePoint Now() const override { return current_time_; }

  /// Sets the clock to a specific time point.
  void SetTime(TimePoint time) { current_time_ = time; }

  /// Advances the clock by the given duration.
  void Advance(std::chrono::nanoseconds duration) { current_time_ += duration; }

 private:
  TimePoint current_time_;
};

}  // namespace beluga_estimation

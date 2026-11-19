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
 * @brief Includes rate interface mock for testing.
 */

#include <chrono>

#include <beluga_estimation/middleware/rate_interface.hpp>
#include <beluga_estimation/mocks/mock_clock.hpp>

namespace beluga_estimation {

/**
 * @brief Mock implementation of RateInterface for testing.
 *
 * Provides a rate limiter that advances a mock clock on each Sleep() call,
 * enabling deterministic time-based testing without actual delays.
 */
class MockRate : public RateInterface {
 public:
  /**
   * @brief Constructs a mock rate with the specified frequency.
   * @param frequency Target frequency in Hz (determines time advancement per sleep).
   * @param clock Optional mock clock to advance on each Sleep() call.
   */
  explicit MockRate(double frequency, MockClock* clock = nullptr) : frequency_(frequency), clock_(clock) {}

  Result Sleep(const ComputeBudget& /*budget*/) override {
    sleep_count_++;
    if (clock_) {
      // Advance clock by the period (1/frequency)
      auto period =
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / frequency_));
      clock_->Advance(period);
    }
    return Ok();
  }

  void Reset() override { reset_count_++; }

  /// Returns the configured frequency.
  double GetFrequency() const { return frequency_; }

  /// Returns the number of times Sleep() was called.
  int SleepCount() const { return sleep_count_; }

  /// Returns the number of times Reset() was called.
  int ResetCount() const { return reset_count_; }

 private:
  double frequency_;
  MockClock* clock_;
  int sleep_count_ = 0;
  int reset_count_ = 0;
};

}  // namespace beluga_estimation

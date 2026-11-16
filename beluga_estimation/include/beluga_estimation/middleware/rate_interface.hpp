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
 * @brief Includes rate interface declarations.
 */

#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/compute/budget.hpp>

namespace beluga_estimation {

/**
 * @brief Abstract interface for rate-limited sleeping.
 *
 * Controls loop execution frequency by sleeping until the next
 * scheduled tick, while respecting an externally provided compute
 * budget (deadline and stop token).
 */
class RateInterface {
 public:
  virtual ~RateInterface() = default;

  /**
   * @brief Sleeps until the next tick, subject to the given budget.
   *
   * Checks the budget before and after sleeping. Returns immediately
   * without sleeping if the budget is already exceeded.
   *
   * @note Budget enforcement is best-effort: the budget is only checked at
   *       entry and exit of the sleep, not during. A deadline or stop request
   *       that occurs mid-sleep will not be detected until the sleep completes.
   *
   * @param budget The compute budget to check against.
   * @return Success if the sleep completed within budget,
   *         an error with `Error::kTimeout` if the deadline was exceeded,
   *         an error with `Error::kInterrupted` if a stop was requested.
   */
  virtual Result Sleep(const ComputeBudget& budget) = 0;

  /**
   * @brief Resets the rate timer to the current time.
   *
   * Call this after a long computation to prevent the rate from trying to
   * catch up on missed ticks in the next Sleep call.
   */
  virtual void Reset() = 0;
};

}  // namespace beluga_estimation

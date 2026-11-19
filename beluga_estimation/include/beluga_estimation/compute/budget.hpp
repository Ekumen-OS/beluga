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
 * @brief Defines the compute budget type used to constrain operation execution.
 */

#include <optional>

#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation {

/// @brief Resource limits for a single compute operation invocation.
///
/// A `ComputeBudget` bundles two orthogonal constraints:
/// - A **deadline** for the operation to execute, after which the budget is
///   exhausted and @ref Check() returns an error with `Error::kTimeout`,
///   defined w.r.t. the middleware clock.
/// - An **allowance** token whose owner can request early termination, after
///   which the budget is exhausted and @ref Check() returns an error with
///   `Error::kInterrupted`.
///
/// Either constraint may be absent (empty `optional` or default-constructed
/// `stop_token`), in which case that particular limit is not enforced.
/// @ref Unlimited provides a ready-made instance with both constraints absent.
class ComputeBudget {
 public:
  /// @brief A budget with no deadline — computation runs until finished or stopped if an allowance is specified.
  static ComputeBudget Unlimited(std_future::stop_token allowance = {}) {
    return ComputeBudget(std::nullopt, std::move(allowance));
  }

  /// @brief Constructs a compute budget with the given deadline and allowance.
  ///
  /// @param deadline Optional wall-clock deadline. If set, calls to @ref Check() after this point
  ///                 return an error with `Error::kTimeout`.
  /// @param allowance Stop token that can be used to interrupt the operation externally. When the
  ///                  associated @ref std_future::stop_source issues a stop request, calls to
  ///                  @ref Check() return an error with `Error::kInterrupted`. A default-constructed
  ///                  token (no associated stop state) never triggers.
  ComputeBudget(std::optional<TimePoint> deadline, std_future::stop_token allowance)
      : deadline_(deadline), allowance_(std::move(allowance)) {}

  /// @brief Returns the optional deadline for this budget.
  const std::optional<TimePoint>& GetDeadline() const noexcept { return deadline_; }

  /// @brief Returns the execution allowance for this budget.
  const std_future::stop_token& GetAllowance() const noexcept { return allowance_; }

  /// @brief Evaluates whether the budget is still valid at a given point in time.
  ///
  /// The check order is:
  /// 1. If a stop has been requested via @ref allowance, returns an error with `Error::kInterrupted`.
  /// 2. If @ref deadline is set and `time` is past it, returns an error with `Error::kTimeout`.
  /// 3. Otherwise returns success.
  ///
  /// @param time The current time to evaluate against the deadline.
  /// @return The resulting `Result`.
  Result Check(TimePoint time) const noexcept {
    if (allowance_.stop_requested()) {
      return Err(Error::kInterrupted);
    }
    if (deadline_ && time > *deadline_) {
      return Err(Error::kTimeout);
    }
    return Ok();
  }

 private:
  std::optional<TimePoint> deadline_;
  std_future::stop_token allowance_;
};

}  // namespace beluga_estimation

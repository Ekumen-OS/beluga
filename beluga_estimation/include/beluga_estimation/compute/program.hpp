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
 * @brief Defines the abstract interface for Bayesian estimation programs.
 */

#include <beluga_estimation/common/registry.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/compute/budget.hpp>

namespace beluga_estimation {

/// @brief Abstract interface for a self-contained Bayesian estimation program.
///
/// A `BayesianProgram` represents a complete, runnable estimation algorithm
/// (e.g. a particle filter or Kalman filter loop) that manages its own
/// internal state and resources.
///
/// Unlike @ref BayesianOperation, which exposes a three-phase lifecycle tied
/// to an external @ref DynamicBayesianNetwork, a `BayesianProgram` is
/// self-contained: a single call to @ref Run() drives the full estimation
/// pipeline.
///
/// Implementations should honour the supplied @ref ComputeBudget and return
/// an error result (e.g. with `Error::kTimeout` or `Error::kInterrupted`) when
/// the budget is exhausted, rather than blocking indefinitely.
///
/// @tparam T Scalar type used internally by the estimation algorithm
///           (e.g. `float` or `double`).
template <typename T>
class BayesianProgram_ {
 public:
  virtual ~BayesianProgram_() = default;

  /// @brief Runs the estimation program with an unlimited compute budget.
  ///
  /// Equivalent to calling `Run(ComputeBudget::Unlimited())`.
  ///
  /// @return The resulting `Result`.
  Result Run() { return Run(ComputeBudget::Unlimited()); };

  /// @brief Runs the estimation program, respecting the given compute budget.
  ///
  /// Implementations should periodically check `budget` and return early with
  /// an error result when the budget is exhausted.
  ///
  /// @param budget Resource limits (deadline and/or stop token) for this run.
  /// @return The resulting `Result`.
  virtual Result Run(const ComputeBudget& budget) = 0;
};

using BayesianProgram = BayesianProgram_<DefaultScalar>;

}  // namespace beluga_estimation

BELUGA_ESTIMATION_EXPORT_TYPE(
    beluga_estimation::BayesianProgram,
    (std::shared_ptr<beluga_estimation::MiddlewareInterface>, std::string_view));

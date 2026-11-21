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
 * @brief Defines the abstract interface for Bayesian compute operations.
 */

#include <beluga_estimation/common/registry.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/compute/budget.hpp>
#include <beluga_estimation/compute/context.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>

namespace beluga_estimation {

/// @brief Abstract interface for a single compute step acting on a Bayesian network.
///
/// A `BayesianOperation` encapsulates one logical computation — such as a
/// prediction, an update, or a resampling pass — that reads from and writes to
/// a @ref DynamicBayesianNetwork.
///
/// Implementations follow a three-phase lifecycle:
/// 1. @ref Initialize() — allocate resources and perform any one-time setup.
/// 2. @ref Execute() — run the computation, optionally bounded by a @ref ComputeBudget.
/// 3. @ref Finalize() — release resources and perform any required cleanup.
///
/// All lifecycle methods return a @ref Result so that callers can rely on
/// structured error reporting rather than exception handling for recoverable
/// failures. Unrecoverable failures should use BELUGA_ESTIMATION_DEMAND.
///
/// @tparam T Scalar type used by the underlying Bayesian network (e.g. `float` or `double`).
template <typename T>
class BayesianOperation_ {
 public:
  virtual ~BayesianOperation_() = default;

  /// @brief Allocates resources and performs one-time setup before execution.
  ///
  /// The default implementation does nothing and returns success.
  /// Override to perform operation-specific initialization.
  ///
  /// @param budget Resource limits (deadline and/or stop token) for initialization.
  /// @return Success, or an error result otherwise.
  virtual Result Initialize(const ComputeBudget& budget) { return Ok(); }

  /// @brief Executes the operation on `context`, respecting the given compute budget.
  ///
  /// Implementations should periodically check `budget` and return early with
  /// an error result when the budget is exhausted.
  ///
  /// @param context The execution context with DBN and estimate store.
  /// @param budget Resource limits (deadline and/or stop token) for this invocation.
  /// @return The resulting `Result`.
  virtual Result Execute(Context_<T>& context, const ComputeBudget& budget) = 0;

  /// @brief Releases resources and performs any required cleanup after execution.
  ///
  /// The default implementation does nothing and returns success.
  /// Override to perform operation-specific teardown.
  ///
  /// @param budget Resource limits (deadline and/or stop token) for finalization.
  /// @return Success, or an error result otherwise.
  virtual Result Finalize(const ComputeBudget& budget) { return Ok(); }
};

using BayesianOperation = BayesianOperation_<DefaultScalar>;

}  // namespace beluga_estimation

BELUGA_ESTIMATION_EXPORT_TYPE(
    beluga_estimation::BayesianOperation,
    (std::shared_ptr<beluga_estimation::MiddlewareInterface>, std::string_view));

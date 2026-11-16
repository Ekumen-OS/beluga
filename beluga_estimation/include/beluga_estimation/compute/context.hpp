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
 * @brief Defines the execution context bundling DBN and estimate store.
 */

#include <beluga_estimation/core/estimate_store.hpp>
#include <beluga_estimation/core/network.hpp>

namespace beluga_estimation {

/// @brief Bundles the resources needed for a single compute operation invocation.
///
/// A `Context` is the single data-flow conduit through which a
/// @ref BayesianOperation_ accesses the DBN and the estimate store.
/// The context is owned by the orchestration layer and passed by reference
/// to each operation's @ref BayesianOperation_::Execute.
///
/// @tparam T Scalar type (e.g. `float` or `double`).
template <typename T>
struct Context_ {
  /// The DBN to operate on (graph topology).
  DynamicBayesianNetwork_<T> dbn;

  /// The estimate store for reading and writing estimates.
  EstimateStore_<T> estimates;
};

using Context = Context_<DefaultScalar>;

}  // namespace beluga_estimation

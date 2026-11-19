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
 * @brief Includes common, widely used types in Beluga.
 */

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <tuple>
#include <type_traits>

#include <beluga_estimation/common/interval.hpp>

namespace beluga_estimation {

/// @brief Default scalar quantity, used throughout Beluga.
///
/// Beluga relies on double precision floating point numbers
/// by default to minimize quantization effects. Modern CPU
/// processors also feature suitable FPUs to handle the
/// computational load.
using DefaultScalar = double;

/// @brief Compile-time dimension tag.
///
/// Tags a dimension known at compile time, so that code which knows its own
/// dimensionality can select statically sized manifolds and distributions
/// without giving up the runtime-dimensioned storage used across the network.
/// Construct one with `NDim<N>{}`.
///
/// @tparam N The dimension.
template <std::size_t N>
using NDim = std::integral_constant<std::size_t, N>;

// Aliases for time quantities
using TimePoint = std::chrono::system_clock::time_point;

using TimeInterval = Interval<TimePoint>;

}  // namespace beluga_estimation

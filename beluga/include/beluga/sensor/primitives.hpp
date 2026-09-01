// Copyright 2025 Ekumen, Inc.
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

#ifndef BELUGA_SENSOR_PRIMITIVES_HPP
#define BELUGA_SENSOR_PRIMITIVES_HPP

#include <type_traits>
#include <utility>
#include <vector>

// /**
//  * \file
//  * \brief Implementation of sensor primitives to abstract member access.
//  */

namespace beluga {

// Primary template which defaults to `false_type`.
// A specialization will override this if the method is detected.
template <class T, class = void>
struct has_likelihood_field : std::false_type {};

// Specialization. Uses SFINAE to detect whether the expression
// `std::declval<T>().likelihood_field()` is valid.
// If so, evaluates to `true_type`.
template <class T>
struct has_likelihood_field<T, std::void_t<decltype(std::declval<T>().likelihood_field())>> : std::true_type {};

/// Trait variable that indicates whether a type `T` has a `likelihood_field()` method.
template <class T>
inline constexpr bool has_likelihood_field_v = has_likelihood_field<T>::value;

// Primary template which defaults to `false_type`.
// A specialization will override this if the method is detected.
template <class T, class = void>
struct has_beam_skip : std::false_type {};

// Specialization. Uses SFINAE to detect whether a sensor model exposes a
// `prepare(measurement, states)` method, used to precompute a beam skipping mask over the
// whole particle set before per-particle reweighting. The states range is probed with a
// `std::vector<state_type>`; any range type is accepted by the actual templated method.
template <class T>
struct has_beam_skip<
    T,
    std::void_t<decltype(std::declval<T&>().prepare(
        std::declval<const typename T::measurement_type&>(),
        std::declval<const std::vector<typename T::state_type>&>()))>> : std::true_type {};

/// Trait variable that indicates whether a type `T` supports beam skipping via `prepare()`.
template <class T>
inline constexpr bool has_beam_skip_v = has_beam_skip<T>::value;

}  // namespace beluga

#endif  // BELUGA_SENSOR_PRIMITIVES_HPP

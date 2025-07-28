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

}  // namespace beluga

#endif  // BELUGA_SENSOR_PRIMITIVES_HPP

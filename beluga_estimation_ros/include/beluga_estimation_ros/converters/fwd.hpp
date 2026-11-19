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

#include <memory>
#include <type_traits>
#include <utility>

#include <beluga_estimation/common/message.hpp>

/**
 * @file
 * @brief Provides a generic framework for converting types between ROS and Beluga.
 *
 * This file defines the `TypeConverter` template, which can be specialized for different
 * types to provide custom conversion logic. It also provides the `ToRos` and `FromRos`
 * functions that leverage these converters.
 */

namespace beluga_estimation_ros {

/**
 * @brief Generic type converter.
 *
 * This struct provides a template for type converters. It is intended to be
 * specialized for different types to provide custom conversion logic.
 *
 * @tparam T The type to convert.
 */
template <typename T>
struct TypeConverter {
  static_assert(
      beluga_estimation::std_future::always_false_v<T>,
      "\n\n"
      "===========================================================================================\n"
      "  BELUGA ESTIMATION ROS ERROR: beluga_estimation_ros::TypeConverter is not specialized for this type.           "
      " \n"
      "  You must provide a specialization to convert between this native type and a ROS message. \n"
      "===========================================================================================\n");
};

/**
 * @brief Converts a value to its ROS representation.
 *
 * This function uses the `TypeConverter` for the given type to convert a value
 * to its ROS representation.
 *
 * @tparam T The type of the value to convert.
 * @param value The value to convert.
 * @return The ROS representation of the value.
 */
template <typename T>
decltype(auto) ToRos(T&& value) {
  return TypeConverter<std::decay_t<T>>::ToRos(std::forward<T>(value));
}

/**
 * @brief Converts a value from its ROS representation.
 *
 * This function uses the `TypeConverter` for the given type to convert a value
 * from its ROS representation.
 *
 * @tparam T The type of the value to convert.
 * @param value The value to convert.
 * @return The Beluga representation of the value.
 */
template <typename T>
decltype(auto) FromRos(T&& value) {
  return TypeConverter<std::decay_t<T>>::FromRos(std::forward<T>(value));
}

}  // namespace beluga_estimation_ros

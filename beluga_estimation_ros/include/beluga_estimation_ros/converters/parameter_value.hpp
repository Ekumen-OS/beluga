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
 * @brief Includes parameter value converter declarations.
 */

#include <rclcpp/parameter_value.hpp>

#include <beluga_estimation/middleware/parameter_value.hpp>
#include <beluga_estimation_ros/converters/fwd.hpp>

namespace beluga_estimation_ros {

/**
 * @brief Specialization of TypeConverter for beluga_estimation::ParameterType.
 */
template <>
struct TypeConverter<beluga_estimation::ParameterType> {
  static rclcpp::ParameterType ToRos(beluga_estimation::ParameterType);
  static beluga_estimation::ParameterType FromRos(rclcpp::ParameterType);
};

template <>
struct TypeConverter<rclcpp::ParameterType> : TypeConverter<beluga_estimation::ParameterType> {};

/**
 * @brief Specialization of TypeConverter for beluga_estimation::ParameterValue.
 */
template <>
struct TypeConverter<beluga_estimation::ParameterValue> {
  static rclcpp::ParameterValue ToRos(const beluga_estimation::ParameterValue&);
  static beluga_estimation::ParameterValue FromRos(const rclcpp::ParameterValue&);
};

template <>
struct TypeConverter<rclcpp::ParameterValue> : TypeConverter<beluga_estimation::ParameterValue> {};

}  // namespace beluga_estimation_ros

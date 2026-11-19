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

#include <beluga_estimation_ros/converters/parameter_value.hpp>

namespace beluga_estimation_ros {

rclcpp::ParameterType TypeConverter<beluga_estimation::ParameterType>::ToRos(beluga_estimation::ParameterType other) {
  switch (other) {
    case beluga_estimation::ParameterType::kBool:
      return rclcpp::ParameterType::PARAMETER_BOOL;

    case beluga_estimation::ParameterType::kInteger:
      return rclcpp::ParameterType::PARAMETER_INTEGER;

    case beluga_estimation::ParameterType::kDouble:
      return rclcpp::ParameterType::PARAMETER_DOUBLE;

    case beluga_estimation::ParameterType::kString:
      return rclcpp::ParameterType::PARAMETER_STRING;

    case beluga_estimation::ParameterType::kBoolArray:
      return rclcpp::ParameterType::PARAMETER_BOOL_ARRAY;

    case beluga_estimation::ParameterType::kIntegerArray:
      return rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;

    case beluga_estimation::ParameterType::kDoubleArray:
      return rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;

    case beluga_estimation::ParameterType::kStringArray:
      return rclcpp::ParameterType::PARAMETER_STRING_ARRAY;

    default:
      return rclcpp::ParameterType::PARAMETER_NOT_SET;
  }
}

beluga_estimation::ParameterType TypeConverter<beluga_estimation::ParameterType>::FromRos(rclcpp::ParameterType other) {
  switch (other) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return beluga_estimation::ParameterType::kBool;

    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return beluga_estimation::ParameterType::kInteger;

    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return beluga_estimation::ParameterType::kDouble;

    case rclcpp::ParameterType::PARAMETER_STRING:
      return beluga_estimation::ParameterType::kString;

    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return beluga_estimation::ParameterType::kBoolArray;

    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return beluga_estimation::ParameterType::kIntegerArray;

    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return beluga_estimation::ParameterType::kDoubleArray;

    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return beluga_estimation::ParameterType::kStringArray;

    default:
      return beluga_estimation::ParameterType::kNotSet;
  }
}

rclcpp::ParameterValue TypeConverter<beluga_estimation::ParameterValue>::ToRos(
    const beluga_estimation::ParameterValue& other) {
  switch (other.GetType()) {
    case beluga_estimation::ParameterType::kBool:
      return rclcpp::ParameterValue{other.As<beluga_estimation::ParameterType::kBool>()};

    case beluga_estimation::ParameterType::kInteger:
      return rclcpp::ParameterValue{other.As<beluga_estimation::ParameterType::kInteger>()};

    case beluga_estimation::ParameterType::kDouble:
      return rclcpp::ParameterValue{other.As<beluga_estimation::ParameterType::kDouble>()};

    case beluga_estimation::ParameterType::kString:
      return rclcpp::ParameterValue{other.As<beluga_estimation::ParameterType::kString>()};

    case beluga_estimation::ParameterType::kBoolArray:
      return rclcpp::ParameterValue{other.As<beluga_estimation::ParameterType::kBoolArray>()};

    case beluga_estimation::ParameterType::kIntegerArray:
      return rclcpp::ParameterValue{other.As<beluga_estimation::ParameterType::kIntegerArray>()};

    case beluga_estimation::ParameterType::kDoubleArray:
      return rclcpp::ParameterValue{other.As<beluga_estimation::ParameterType::kDoubleArray>()};

    case beluga_estimation::ParameterType::kStringArray:
      return rclcpp::ParameterValue{other.As<beluga_estimation::ParameterType::kStringArray>()};

    default:
      return rclcpp::ParameterValue{};
  }
}

beluga_estimation::ParameterValue TypeConverter<beluga_estimation::ParameterValue>::FromRos(
    const rclcpp::ParameterValue& other) {
  switch (other.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return beluga_estimation::ParameterValue{other.get<rclcpp::ParameterType::PARAMETER_BOOL>()};

    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return beluga_estimation::ParameterValue{other.get<rclcpp::ParameterType::PARAMETER_INTEGER>()};

    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return beluga_estimation::ParameterValue{other.get<rclcpp::ParameterType::PARAMETER_DOUBLE>()};

    case rclcpp::ParameterType::PARAMETER_STRING:
      return beluga_estimation::ParameterValue{other.get<rclcpp::ParameterType::PARAMETER_STRING>()};

    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return beluga_estimation::ParameterValue{other.get<rclcpp::ParameterType::PARAMETER_BOOL_ARRAY>()};

    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return beluga_estimation::ParameterValue{other.get<rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY>()};

    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return beluga_estimation::ParameterValue{other.get<rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY>()};

    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return beluga_estimation::ParameterValue{other.get<rclcpp::ParameterType::PARAMETER_STRING_ARRAY>()};

    default:
      return beluga_estimation::ParameterValue{};
  }
}

}  // namespace beluga_estimation_ros

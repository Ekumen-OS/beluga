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

#include <fmt/format.h>

#include <beluga_estimation/middleware/parameter_value.hpp>

namespace beluga_estimation {

std::string_view ToStringView(ParameterType type) {
  switch (type) {
    case ParameterType::kNotSet:
      return "NOT_SET";
    case ParameterType::kBool:
      return "BOOL";
    case ParameterType::kInteger:
      return "INTEGER";
    case ParameterType::kDouble:
      return "DOUBLE";
    case ParameterType::kString:
      return "STRING";
    case ParameterType::kBoolArray:
      return "BOOL_ARRAY";
    case ParameterType::kIntegerArray:
      return "INTEGER_ARRAY";
    case ParameterType::kDoubleArray:
      return "DOUBLE_ARRAY";
    case ParameterType::kStringArray:
      return "STRING_ARRAY";
    default:
      return "UNKNOWN";
  }
}

std::string ToString(ParameterType type) {
  return std::string(ToStringView(type));
}

ParameterTypeException::ParameterTypeException(ParameterType expected, ParameterType actual)
    : std::runtime_error(fmt::format("expected [{}] got [{}]", ToString(expected), ToString(actual))) {}

ParameterType ParameterValue::GetType() const {
  return static_cast<ParameterType>(data_.index());
};

std::string_view ParameterValue::GetTypeName() const {
  return ToStringView(GetType());
}

}  // namespace beluga_estimation

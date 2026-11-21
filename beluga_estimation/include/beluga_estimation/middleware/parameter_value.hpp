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
 * @brief Includes parameter value declarations.
 */

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <beluga_estimation/common/std_future.hpp>

namespace beluga_estimation {

/// Enumeration of supported parameter types.
enum class ParameterType : std::size_t {
  kNotSet,
  kBool,
  kInteger,
  kDouble,
  kString,
  kBoolArray,
  kIntegerArray,
  kDoubleArray,
  kStringArray,
};

/// Variant type that can hold any of the supported parameter values.
using ParameterVariant = std::variant<
    std::monostate,
    bool,
    std::int64_t,
    double,
    std::string,
    std::vector<bool>,
    std::vector<std::int64_t>,
    std::vector<double>,
    std::vector<std::string>>;

/**
 * @brief Maps a C++ type to its corresponding ParameterType enum value.
 * @tparam T The C++ type to map.
 * @return The corresponding ParameterType.
 */
template <typename T>
constexpr ParameterType GetParameterType() {
  if constexpr (std::is_same_v<T, std::monostate>) {
    return ParameterType::kNotSet;
  } else if constexpr (std::is_same_v<T, bool>) {
    return ParameterType::kBool;
  } else if constexpr (std::is_same_v<T, std::int64_t>) {
    return ParameterType::kInteger;
  } else if constexpr (std::is_same_v<T, double>) {
    return ParameterType::kDouble;
  } else if constexpr (std::is_same_v<T, std::string>) {
    return ParameterType::kString;
  } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
    return ParameterType::kBoolArray;
  } else if constexpr (std::is_same_v<T, std::vector<std::int64_t>>) {
    return ParameterType::kIntegerArray;
  } else if constexpr (std::is_same_v<T, std::vector<double>>) {
    return ParameterType::kDoubleArray;
  } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
    return ParameterType::kStringArray;
  } else {
    static_assert(std::is_same_v<T, void>, "Unsupported parameter type");
  }
}

/// Converts a ParameterType to its string representation as a string_view.
std::string_view ToStringView(ParameterType type);

/// Converts a ParameterType to its string representation as a string.
std::string ToString(ParameterType type);

/// Exception thrown when a parameter access is attempted with the wrong type.
class ParameterTypeException : public std::runtime_error {
 public:
  /**
   * @brief Constructs a ParameterTypeException.
   * @param expected The expected parameter type.
   * @param actual The actual parameter type found.
   */
  ParameterTypeException(ParameterType expected, ParameterType actual);
};

/**
 * @brief A type-safe container for parameter values.
 *
 * This class wraps a variant of supported parameter types and provides
 * a convenient interface for type-safe access and conversion.
 */
class ParameterValue {
 public:
  /// Constructs an empty parameter value (kNotSet).
  ParameterValue() = default;

  /// Constructs a boolean parameter value.
  explicit ParameterValue(bool value) : data_(value) {}

  /// Constructs an integer parameter value from a 32-bit integer.
  explicit ParameterValue(std::int32_t value) : data_(static_cast<std::int64_t>(value)) {}

  /// Constructs an integer parameter value from a 64-bit integer.
  explicit ParameterValue(std::int64_t value) : data_(value) {}

  /// Constructs a double parameter value.
  explicit ParameterValue(double value) : data_(value) {}

  /// Constructs a string parameter value from a string_view.
  explicit ParameterValue(std::string_view value) : data_(std::string(value)) {}

  /// Constructs a string parameter value from a C-style string.
  explicit ParameterValue(const char* value) : data_(std::string(value)) {}

  /// Constructs a boolean array parameter value.
  explicit ParameterValue(std::vector<bool> value) : data_(std::move(value)) {}

  /// Constructs an integer array parameter value.
  explicit ParameterValue(std::vector<std::int64_t> value) : data_(std::move(value)) {}

  /// Constructs a double array parameter value.
  explicit ParameterValue(std::vector<double> value) : data_(std::move(value)) {}

  /// Constructs a string array parameter value.
  explicit ParameterValue(std::vector<std::string> value) : data_(std::move(value)) {}

  /// Gets the type of the stored parameter value.
  ParameterType GetType() const;

  /// Gets the name of the type of the stored parameter value.
  std::string_view GetTypeName() const;

  /**
   * @brief Accesses the stored value as a specific C++ type.
   * @tparam T The expected C++ type.
   * @return A reference to the stored value.
   * @throws ParameterTypeException if the stored type does not match T.
   */
  template <typename T>
  const auto& As() const {
    return As<GetParameterType<T>()>();
  }

  /**
   * @brief Accesses the stored value as a specific ParameterType.
   * @tparam I The expected ParameterType.
   * @return A reference to the stored value.
   * @throws ParameterTypeException if the stored type does not match I.
   */
  template <ParameterType I>
  const auto& As() const {
    try {
      return std::get<std_future::to_underlying(I)>(data_);
    } catch (const std::bad_variant_access&) {
      throw ParameterTypeException(I, GetType());
    }
  }

  /// Accesses the value as a boolean.
  bool AsBool() const { return As<bool>(); }

  /// Accesses the value as a 64-bit integer.
  std::int64_t AsInt() const { return As<std::int64_t>(); }

  /// Accesses the value as a double.
  double AsDouble() const { return As<double>(); }

  /// Accesses the value as a string.
  const std::string& AsString() const { return As<std::string>(); }

  /// Accesses the value as a boolean array.
  const std::vector<bool>& AsBoolArray() const { return As<std::vector<bool>>(); }

  /// Accesses the value as an integer array.
  const std::vector<std::int64_t>& AsIntArray() const { return As<std::vector<std::int64_t>>(); }

  /// Accesses the value as a double array.
  const std::vector<double>& AsDoubleArray() const { return As<std::vector<double>>(); }

  /// Accesses the value as a string array.
  const std::vector<std::string>& AsStringArray() const { return As<std::vector<std::string>>(); }

  friend bool operator==(const ParameterValue& lhs, const ParameterValue& rhs) { return lhs.data_ == rhs.data_; }

  friend bool operator!=(const ParameterValue& lhs, const ParameterValue& rhs) { return lhs.data_ != rhs.data_; }

 private:
  ParameterVariant data_;
};

}  // namespace beluga_estimation

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
 * @brief Includes parameter interface declarations.
 */

#include <string>
#include <string_view>

#include <beluga_estimation/middleware/parameter_value.hpp>

namespace beluga_estimation {

/**
 * @brief Abstract interface for parameter management.
 *
 * This class provides a unified interface to declare, get, and check for
 * parameters across different middleware backends.
 */
class ParametersInterface {
 public:
  virtual ~ParametersInterface() = default;

  /**
   * @brief Declares a parameter without a default value.
   * @tparam T The type of the parameter.
   * @param name The name of the parameter.
   * @return The value of the declared parameter.
   */
  template <typename T>
  T DeclareParameter(std::string_view name) {
    return DeclareParameter(name, GetParameterType<T>()).template As<T>();
  }

  /**
   * @brief Declares a parameter with a default value.
   * @tparam T The type of the parameter.
   * @param name The name of the parameter.
   * @param default_value The default value to use if the parameter is not set.
   * @return The value of the declared parameter.
   */
  template <typename T>
  T DeclareParameter(std::string_view name, T default_value) {
    return DeclareParameter(name, ParameterValue(std::move(default_value))).template As<T>();
  }

  /**
   * @brief Gets the value of a parameter.
   * @tparam T The type of the parameter.
   * @param name The name of the parameter.
   * @return The value of the parameter.
   */
  template <typename T>
  T GetParameter(std::string_view name) const {
    return GetParameter(name).template As<T>();
  }

  /**
   * @brief Declares a parameter with a specific type.
   * @param name The name of the parameter.
   * @param type The expected type of the parameter.
   * @return The value of the declared parameter.
   */
  virtual ParameterValue DeclareParameter(std::string_view name, ParameterType type) = 0;

  /**
   * @brief Declares a parameter with a default value.
   * @param name The name of the parameter.
   * @param default_value The default value to use.
   * @return The value of the declared parameter.
   */
  virtual ParameterValue DeclareParameter(std::string_view name, const ParameterValue& default_value) = 0;

  /**
   * @brief Gets the value of a parameter.
   * @param name The name of the parameter.
   * @return The value of the parameter.
   */
  virtual ParameterValue GetParameter(std::string_view name) const = 0;

  /// Checks if a parameter is declared.
  virtual bool HasParameter(std::string_view name) const = 0;
};

}  // namespace beluga_estimation

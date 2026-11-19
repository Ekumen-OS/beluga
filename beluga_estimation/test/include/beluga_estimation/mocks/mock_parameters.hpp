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
 * @brief Includes parameter interface mock.
 */

#include <gmock/gmock.h>

#include <beluga_estimation/middleware/parameters_interface.hpp>

namespace beluga_estimation {

class MockParameters : public ParametersInterface {
 public:
  using ParametersInterface::DeclareParameter;
  using ParametersInterface::GetParameter;

  MOCK_METHOD(ParameterValue, DeclareParameter, (std::string_view, ParameterType), (override));

  MOCK_METHOD(ParameterValue, DeclareParameter, (std::string_view, const ParameterValue&), (override));

  MOCK_METHOD(ParameterValue, GetParameter, (std::string_view), (override, const));

  MOCK_METHOD(bool, HasParameter, (std::string_view), (override, const));
};

}  // namespace beluga_estimation

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
 * @brief Includes logger interface mock.
 */

#include <gmock/gmock.h>

#include <beluga_estimation/middleware/logger_interface.hpp>

namespace beluga_estimation {

class MockLogger : public LoggerInterface {
 public:
  /// Constructs a mock logger with every severity enabled by default.
  MockLogger() { ON_CALL(*this, IsEnabled(::testing::_)).WillByDefault(::testing::Return(true)); }

  MOCK_METHOD(void, Log, (LogLevel level, std::string_view message), (override));
  MOCK_METHOD(bool, IsEnabled, (LogLevel level), (const, override));
};

}  // namespace beluga_estimation

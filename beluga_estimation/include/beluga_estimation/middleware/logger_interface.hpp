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

#include <string_view>

namespace beluga_estimation {

/// Severity levels for logging.
enum class LogLevel { kDebug, kInfo, kWarn, kError, kFatal };

/// Abstract interface for logging operations.
class LoggerInterface {
 public:
  virtual ~LoggerInterface() = default;

  /// Logs a message with the specified severity level.
  /// @param level The severity level of the log message.
  /// @param message The message to be logged.
  virtual void Log(LogLevel level, std::string_view message) = 0;

  /**
   * @brief Checks whether messages at the given severity level would be emitted.
   *
   * Callers should consult this before doing any work solely to produce a
   * message, such as formatting its arguments. Note that function arguments are
   * evaluated before the call, so gating only avoids the formatting and logging
   * work, not the evaluation of the arguments themselves.
   *
   * The default implementation returns `true` to preserve the behavior of
   * logger implementations that predate this method. Implementations that can
   * filter messages should override it.
   *
   * @param level The severity level to query.
   * @return `true` if messages at @p level would be emitted, `false` otherwise.
   */
  virtual bool IsEnabled(LogLevel /*level*/) const { return true; }
};

}  // namespace beluga_estimation

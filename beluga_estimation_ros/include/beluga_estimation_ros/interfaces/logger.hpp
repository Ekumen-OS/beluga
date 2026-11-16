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

#include <rcutils/logging.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>

#include <beluga_estimation/middleware/logger_interface.hpp>

namespace beluga_estimation_ros {

/// ROS 2 implementation of the Beluga logger interface.
class Logger : public beluga_estimation::LoggerInterface {
 public:
  /// Constructs a Logger instance.
  /// @param node_logging The ROS 2 node logging interface to wrap.
  explicit Logger(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging)
      : logger_(node_logging->get_logger()) {}

  /// Logs a message using the underlying ROS 2 logger.
  void Log(beluga_estimation::LogLevel level, std::string_view message) override {
    switch (level) {
      case beluga_estimation::LogLevel::kDebug:
        RCLCPP_DEBUG(logger_, "%.*s", static_cast<int>(message.size()), message.data());
        break;
      case beluga_estimation::LogLevel::kInfo:
        RCLCPP_INFO(logger_, "%.*s", static_cast<int>(message.size()), message.data());
        break;
      case beluga_estimation::LogLevel::kWarn:
        RCLCPP_WARN(logger_, "%.*s", static_cast<int>(message.size()), message.data());
        break;
      case beluga_estimation::LogLevel::kError:
        RCLCPP_ERROR(logger_, "%.*s", static_cast<int>(message.size()), message.data());
        break;
      case beluga_estimation::LogLevel::kFatal:
        RCLCPP_FATAL(logger_, "%.*s", static_cast<int>(message.size()), message.data());
        break;
    }
  }

  /// Checks whether the underlying ROS 2 logger would emit messages at @p level.
  ///
  /// Accounts for both the effective runtime severity of the logger and the
  /// severity levels compiled out by the ROS 2 logging macros.
  bool IsEnabled(beluga_estimation::LogLevel level) const override {
    if (SeverityIndex(level) < RCLCPP_LOG_MIN_SEVERITY || SeverityIndex(level) < RCUTILS_LOG_MIN_SEVERITY) {
      return false;
    }
    return rcutils_logging_logger_is_enabled_for(logger_.get_name(), Severity(level));
  }

  /// Maps a Beluga severity level to a ROS 2 compile-time severity index.
  /// @param level The Beluga severity level to map.
  /// @return The matching `RCLCPP_LOG_MIN_SEVERITY_*` index.
  static constexpr int SeverityIndex(beluga_estimation::LogLevel level) {
    switch (level) {
      case beluga_estimation::LogLevel::kDebug:
        return RCLCPP_LOG_MIN_SEVERITY_DEBUG;
      case beluga_estimation::LogLevel::kInfo:
        return RCLCPP_LOG_MIN_SEVERITY_INFO;
      case beluga_estimation::LogLevel::kWarn:
        return RCLCPP_LOG_MIN_SEVERITY_WARN;
      case beluga_estimation::LogLevel::kError:
        return RCLCPP_LOG_MIN_SEVERITY_ERROR;
      case beluga_estimation::LogLevel::kFatal:
        return RCLCPP_LOG_MIN_SEVERITY_FATAL;
    }
    return RCLCPP_LOG_MIN_SEVERITY_FATAL;
  }

  /// Maps a Beluga severity level to a rcutils severity level.
  /// @param level The Beluga severity level to map.
  /// @return The matching `RCUTILS_LOG_SEVERITY_*` value.
  static constexpr int Severity(beluga_estimation::LogLevel level) {
    switch (level) {
      case beluga_estimation::LogLevel::kDebug:
        return RCUTILS_LOG_SEVERITY_DEBUG;
      case beluga_estimation::LogLevel::kInfo:
        return RCUTILS_LOG_SEVERITY_INFO;
      case beluga_estimation::LogLevel::kWarn:
        return RCUTILS_LOG_SEVERITY_WARN;
      case beluga_estimation::LogLevel::kError:
        return RCUTILS_LOG_SEVERITY_ERROR;
      case beluga_estimation::LogLevel::kFatal:
        return RCUTILS_LOG_SEVERITY_FATAL;
    }
    return RCUTILS_LOG_SEVERITY_FATAL;
  }

 private:
  rclcpp::Logger logger_;
};

}  // namespace beluga_estimation_ros

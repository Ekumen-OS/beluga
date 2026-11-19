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
 * @brief Includes middleware interface declarations.
 */

#include <memory>
#include <string_view>
#include <type_traits>
#include <utility>

#include <fmt/format.h>

#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/middleware/clock_interface.hpp>
#include <beluga_estimation/middleware/logger_interface.hpp>
#include <beluga_estimation/middleware/parameters_interface.hpp>
#include <beluga_estimation/middleware/rate_interface.hpp>
#include <beluga_estimation/middleware/topics_interface.hpp>
#include <beluga_estimation/middleware/transform_interface.hpp>

namespace beluga_estimation {

/**
 * @brief Abstract interface for middleware backends.
 *
 * This class provides a unified interface to interact with different middleware
 * implementations (e.g., ROS 2). It aggregates specialized interfaces for
 * parameters, topic-based communication, clock access, and rate-limited execution.
 */
class MiddlewareInterface {
 public:
  virtual ~MiddlewareInterface() = default;

  /// Gets the name of the middleware backend.
  virtual std::string_view GetBackendName() const = 0;

  /**
   * @brief Gets the underlying middleware object.
   *
   * The middleware implementation is free to return any relevant object here,
   * providing an escape hatch for implementing middleware-aware nodes.
   *
   * This pointer should not be used above the middleware interface abstraction layer.
   *
   * @return An opaque pointer to the backend-specific object (e.g., rclcpp::Node*).
   */
  virtual void* GetBackendObject() = 0;

  /// Gets the interface for parameter management.
  virtual beluga_estimation::ParametersInterface* GetParametersInterface() = 0;

  /// Gets the interface for topics management.
  virtual beluga_estimation::TopicsInterface* GetTopicsInterface() = 0;

  /// Gets the clock interface for querying the current time.
  virtual ClockInterface* GetClockInterface() = 0;

  /// Gets the transform interface for querying and publishing transforms.
  virtual TransformInterface* GetTransformInterface() = 0;

  /**
   * @brief Creates a rate limiter at the given frequency.
   * @param frequency The desired loop frequency in Hz.
   * @return A unique pointer to the created rate interface.
   */
  virtual std::unique_ptr<RateInterface> CreateRate(double frequency) = 0;
  /// Gets the interface for logging.
  virtual beluga_estimation::LoggerInterface* GetLoggerInterface() = 0;

  /// Logs a debug message.
  /// @param message The message to be logged.
  void Debug(std::string_view message) { LogMessage(beluga_estimation::LogLevel::kDebug, message); }

  /**
   * @brief Logs a formatted debug message.
   *
   * The format arguments are only formatted and logged when debug logging is
   * enabled. They are still evaluated before the call, as usual in C++.
   *
   * @param format The `fmt` format string.
   * @param args The format arguments.
   */
  template <typename... Args, typename = std::enable_if_t<(sizeof...(Args) > 0)>>
  void Debug(fmt::format_string<Args...> format, Args&&... args) {
    LogFormatted(beluga_estimation::LogLevel::kDebug, format, std::forward<Args>(args)...);
  }

  /// Logs an info message.
  /// @param message The message to be logged.
  void Info(std::string_view message) { LogMessage(beluga_estimation::LogLevel::kInfo, message); }

  /**
   * @brief Logs a formatted info message.
   *
   * The format arguments are only formatted and logged when info logging is
   * enabled. They are still evaluated before the call, as usual in C++.
   *
   * @param format The `fmt` format string.
   * @param args The format arguments.
   */
  template <typename... Args, typename = std::enable_if_t<(sizeof...(Args) > 0)>>
  void Info(fmt::format_string<Args...> format, Args&&... args) {
    LogFormatted(beluga_estimation::LogLevel::kInfo, format, std::forward<Args>(args)...);
  }

  /// Logs a warning message.
  /// @param message The message to be logged.
  void Warn(std::string_view message) { LogMessage(beluga_estimation::LogLevel::kWarn, message); }

  /**
   * @brief Logs a formatted warning message.
   *
   * The format arguments are only formatted and logged when warning logging is
   * enabled. They are still evaluated before the call, as usual in C++.
   *
   * @param format The `fmt` format string.
   * @param args The format arguments.
   */
  template <typename... Args, typename = std::enable_if_t<(sizeof...(Args) > 0)>>
  void Warn(fmt::format_string<Args...> format, Args&&... args) {
    LogFormatted(beluga_estimation::LogLevel::kWarn, format, std::forward<Args>(args)...);
  }

  /// Logs an error message.
  /// @param message The message to be logged.
  void Error(std::string_view message) { LogMessage(beluga_estimation::LogLevel::kError, message); }

  /**
   * @brief Logs a formatted error message.
   *
   * The format arguments are only formatted and logged when error logging is
   * enabled. They are still evaluated before the call, as usual in C++.
   *
   * @param format The `fmt` format string.
   * @param args The format arguments.
   */
  template <typename... Args, typename = std::enable_if_t<(sizeof...(Args) > 0)>>
  void Error(fmt::format_string<Args...> format, Args&&... args) {
    LogFormatted(beluga_estimation::LogLevel::kError, format, std::forward<Args>(args)...);
  }

  /// Logs a fatal message.
  /// @param message The message to be logged.
  void Fatal(std::string_view message) { LogMessage(beluga_estimation::LogLevel::kFatal, message); }

  /**
   * @brief Logs a formatted fatal message.
   *
   * The format arguments are only formatted and logged when fatal logging is
   * enabled. They are still evaluated before the call, as usual in C++.
   *
   * @param format The `fmt` format string.
   * @param args The format arguments.
   */
  template <typename... Args, typename = std::enable_if_t<(sizeof...(Args) > 0)>>
  void Fatal(fmt::format_string<Args...> format, Args&&... args) {
    LogFormatted(beluga_estimation::LogLevel::kFatal, format, std::forward<Args>(args)...);
  }

  /**
   * @brief Declares a parameter without a default value.
   * @tparam T The type of the parameter.
   * @param name The name of the parameter.
   * @return The value of the declared parameter.
   */
  template <typename T>
  T DeclareParameter(std::string_view name) {
    return GetParametersInterface()->DeclareParameter<T>(name);
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
    return GetParametersInterface()->DeclareParameter<T>(name, std::move(default_value));
  }

  /**
   * @brief Gets the value of a parameter.
   * @tparam T The type of the parameter.
   * @param name The name of the parameter.
   * @return The value of the parameter.
   */
  template <typename T>
  T GetParameter(std::string_view name) const {
    return std_future::as_mutable(this)->GetParametersInterface()->GetParameter<T>(name);
  }

  /// Checks if a parameter is declared.
  bool HasParameter(std::string_view name) const {
    return std_future::as_mutable(this)->GetParametersInterface()->HasParameter(name);
  }

  /**
   * @brief Creates a publisher for a specific message type.
   * @tparam T The type of the message to publish.
   * @param topic The name of the topic to publish to.
   * @return The created publisher, or an error if the message type is not available.
   */
  template <typename T>
  Result_<std::shared_ptr<Publisher<T>>> CreatePublisher(std::string_view topic) {
    return GetTopicsInterface()->CreatePublisher<T>(topic);
  }

  /**
   * @brief Creates a subscriber for a specific message type.
   *
   * This method registers a callback that will be invoked whenever a new message
   * of type T is received on the specified topic.
   *
   * @tparam T The type of the message to subscribe to.
   * @param topic The name of the topic to subscribe to.
   * @param callback The function to call when a message is received.
   * @return The created subscriber, or an error if the message type is not available.
   */
  template <typename T>
  Result_<std::shared_ptr<Subscriber<T>>> CreateSubscriber(std::string_view topic, SubscriberCallback<T> callback) {
    return GetTopicsInterface()->CreateSubscriber<T>(topic, std::move(callback));
  }

 private:
  /// Logs a plain message if @p level is enabled.
  void LogMessage(LogLevel level, std::string_view message) {
    auto* logger = GetLoggerInterface();
    if (logger->IsEnabled(level)) {
      logger->Log(level, message);
    }
  }

  /// Formats and logs a message only if @p level is enabled.
  template <typename... Args>
  void LogFormatted(LogLevel level, fmt::format_string<Args...> format, Args&&... args) {
    auto* logger = GetLoggerInterface();
    if (logger->IsEnabled(level)) {
      logger->Log(level, fmt::format(format, std::forward<Args>(args)...));
    }
  }
};

}  // namespace beluga_estimation

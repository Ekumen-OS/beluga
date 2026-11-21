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

#include <array>

#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <rclcpp/rclcpp.hpp>

#include <beluga_estimation_ros/interfaces/logger.hpp>

namespace {

constexpr const char* kLoggerName = "beluga_logger_test";

/// Whether the ROS 2 logging macros emit messages at @p level in this build.
constexpr bool IsCompiledIn(beluga_estimation::LogLevel level) {
  return beluga_estimation_ros::Logger::SeverityIndex(level) >= RCLCPP_LOG_MIN_SEVERITY &&
         beluga_estimation_ros::Logger::SeverityIndex(level) >= RCUTILS_LOG_MIN_SEVERITY;
}

class LoggerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>(kLoggerName);
    logger_ = std::make_unique<beluga_estimation_ros::Logger>(node_->get_node_logging_interface());
    // Restore the process-wide logger level on teardown, even if a test fails.
    previous_level_ = rcutils_logging_get_logger_level(kLoggerName);
  }

  void TearDown() override {
    EXPECT_EQ(rcutils_logging_set_logger_level(kLoggerName, previous_level_), RCUTILS_RET_OK);
    logger_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void SetLevel(int level) { ASSERT_EQ(rcutils_logging_set_logger_level(kLoggerName, level), RCUTILS_RET_OK); }

  static constexpr std::array<beluga_estimation::LogLevel, 5> kLevels = {
      beluga_estimation::LogLevel::kDebug, beluga_estimation::LogLevel::kInfo,  beluga_estimation::LogLevel::kWarn,
      beluga_estimation::LogLevel::kError, beluga_estimation::LogLevel::kFatal,
  };

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<beluga_estimation_ros::Logger> logger_;
  int previous_level_ = RCUTILS_LOG_SEVERITY_UNSET;
};

TEST_F(LoggerTest, RuntimeLevelControlsEnablement) {
  for (const auto runtime_level : {RCUTILS_LOG_SEVERITY_DEBUG, RCUTILS_LOG_SEVERITY_INFO, RCUTILS_LOG_SEVERITY_ERROR}) {
    SetLevel(runtime_level);

    for (const auto level : kLevels) {
      const bool expected = IsCompiledIn(level) && beluga_estimation_ros::Logger::Severity(level) >= runtime_level;
      EXPECT_EQ(logger_->IsEnabled(level), expected)
          << "runtime level " << runtime_level << ", queried level " << static_cast<int>(level);
    }
  }
}

TEST_F(LoggerTest, DebugIsSuppressedWhenFilteredAtRuntime) {
  SetLevel(RCUTILS_LOG_SEVERITY_INFO);

  EXPECT_FALSE(logger_->IsEnabled(beluga_estimation::LogLevel::kDebug));
}

TEST_F(LoggerTest, AllLevelsFollowTheCompileTimeConfiguration) {
  SetLevel(RCUTILS_LOG_SEVERITY_DEBUG);

  for (const auto level : kLevels) {
    EXPECT_EQ(logger_->IsEnabled(level), IsCompiledIn(level));
  }
}

TEST_F(LoggerTest, LoggingAtEveryLevelDoesNotThrow) {
  SetLevel(RCUTILS_LOG_SEVERITY_DEBUG);

  EXPECT_NO_THROW(logger_->Log(beluga_estimation::LogLevel::kDebug, "debug message"));
  EXPECT_NO_THROW(logger_->Log(beluga_estimation::LogLevel::kInfo, "info message"));
  EXPECT_NO_THROW(logger_->Log(beluga_estimation::LogLevel::kWarn, "warn message"));
  EXPECT_NO_THROW(logger_->Log(beluga_estimation::LogLevel::kError, "error message"));
  EXPECT_NO_THROW(logger_->Log(beluga_estimation::LogLevel::kFatal, "fatal message"));
}

}  // namespace

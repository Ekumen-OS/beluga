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

#include <cstddef>
#include <string>
#include <string_view>
#include <utility>

#include <fmt/format.h>
#include <gmock/gmock.h>

#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/mocks/mock_message.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>
#include <beluga_estimation/mocks/mock_publisher.hpp>
#include <beluga_estimation/mocks/mock_subscriber.hpp>

/// A value whose formatting is observable, used to prove formatting is lazy.
struct CountingFormatValue {
  int value;
  static std::size_t format_count;
};

std::size_t CountingFormatValue::format_count = 0;

template <>
struct fmt::formatter<CountingFormatValue> : fmt::formatter<int> {
  template <typename FormatContext>
  auto format(const CountingFormatValue& value, FormatContext& context) const {
    ++CountingFormatValue::format_count;
    return fmt::formatter<int>::format(value.value, context);
  }
};

namespace {

using beluga_estimation::Message;
using beluga_estimation::MessageId;
using beluga_estimation::MockMessage;
using beluga_estimation::ParameterValue;
using beluga_estimation::SubscriberCallback;
using testing::_;
using testing::An;
using testing::Eq;
using testing::Return;

class DummyLogger : public beluga_estimation::LoggerInterface {
 public:
  void Log(beluga_estimation::LogLevel, std::string_view) override {}
};

TEST(MiddlewareInterfaceTest, GetBackendName) {
  beluga_estimation::MockMiddleware mock;
  EXPECT_CALL(mock, GetBackendName()).WillOnce(Return("ROS2"));
  EXPECT_EQ(mock.GetBackendName(), "ROS2");
}

TEST(MiddlewareInterfaceTest, GetBackendObject) {
  beluga_estimation::MockMiddleware mock;
  int dummy_backend = 42;
  EXPECT_CALL(mock, GetBackendObject()).WillOnce(Return(&dummy_backend));
  EXPECT_EQ(mock.GetBackendObject(), &dummy_backend);
}

TEST(MiddlewareInterfaceTest, GetLoggerInterface) {
  beluga_estimation::MockMiddleware mock;
  EXPECT_EQ(mock.GetLoggerInterface(), mock.logger.get());
}

TEST(MiddlewareInterfaceTest, TemplateDeclareParameter) {
  beluga_estimation::MockMiddleware mock;

  EXPECT_CALL(*mock.parameters, DeclareParameter(Eq("test_param"), An<const ParameterValue&>()))
      .WillOnce(Return(ParameterValue(123)));

  EXPECT_EQ(mock.DeclareParameter<std::int64_t>("test_param", 0), 123);
}

TEST(MiddlewareInterfaceTest, TemplateGetParameter) {
  beluga_estimation::MockMiddleware mock;

  EXPECT_CALL(*mock.parameters, GetParameter(Eq("test_param"))).WillOnce(Return(ParameterValue(0.5)));

  EXPECT_DOUBLE_EQ(mock.GetParameter<double>("test_param"), 0.5);
}

TEST(MiddlewareInterfaceTest, CreatePublisherForwardsToTopicsInterface) {
  beluga_estimation::MockMiddleware mock;
  auto expected_publisher = std::make_shared<beluga_estimation::MockPublisher<MockMessage>>();

  EXPECT_CALL(*mock.topics, CreatePublisherImpl(Eq(MockMessage::GetStaticId()), Eq("test_topic")))
      .WillOnce(Return(expected_publisher));

  auto publisher = mock.CreatePublisher<MockMessage>("test_topic");
  EXPECT_EQ(publisher, expected_publisher);
}

TEST(LoggerInterfaceTest, IsEnabledDefaultsToTrue) {
  DummyLogger logger;
  EXPECT_TRUE(logger.IsEnabled(beluga_estimation::LogLevel::kDebug));
  EXPECT_TRUE(logger.IsEnabled(beluga_estimation::LogLevel::kInfo));
  EXPECT_TRUE(logger.IsEnabled(beluga_estimation::LogLevel::kWarn));
  EXPECT_TRUE(logger.IsEnabled(beluga_estimation::LogLevel::kError));
  EXPECT_TRUE(logger.IsEnabled(beluga_estimation::LogLevel::kFatal));
}

TEST(MiddlewareLoggingTest, PlainMessagesForwardedAtEveryLevel) {
  beluga_estimation::MockMiddleware mock;
  const auto plain_message = std::string_view{"plain message"};

  for (const auto level :
       {beluga_estimation::LogLevel::kDebug, beluga_estimation::LogLevel::kInfo, beluga_estimation::LogLevel::kWarn,
        beluga_estimation::LogLevel::kError, beluga_estimation::LogLevel::kFatal}) {
    EXPECT_CALL(*mock.logger, Log(level, plain_message)).Times(1);
  }

  mock.Debug(plain_message);
  mock.Info(plain_message);
  mock.Warn(plain_message);
  mock.Error(plain_message);
  mock.Fatal(plain_message);
}

TEST(MiddlewareLoggingTest, FormattedMessagesForwardedAtEveryLevel) {
  beluga_estimation::MockMiddleware mock;
  const auto expected = std::string_view{"value 42"};

  for (const auto level :
       {beluga_estimation::LogLevel::kDebug, beluga_estimation::LogLevel::kInfo, beluga_estimation::LogLevel::kWarn,
        beluga_estimation::LogLevel::kError, beluga_estimation::LogLevel::kFatal}) {
    EXPECT_CALL(*mock.logger, Log(level, expected)).Times(1);
  }

  mock.Debug("value {}", 42);
  mock.Info("value {}", 42);
  mock.Warn("value {}", 42);
  mock.Error("value {}", 42);
  mock.Fatal("value {}", 42);
}

TEST(MiddlewareLoggingTest, DisabledLevelDoesNotForward) {
  beluga_estimation::MockMiddleware mock;
  ON_CALL(*mock.logger, IsEnabled(::testing::_)).WillByDefault(Return(false));
  EXPECT_CALL(*mock.logger, Log(::testing::_, ::testing::_)).Times(0);

  mock.Debug("debug {}", 1);
  mock.Info("info");
  mock.Warn("warn {}", 3);
  mock.Error("error");
  mock.Fatal("fatal {}", 5);
}

TEST(MiddlewareLoggingTest, DisabledFormattedLogSkipsFormatting) {
  beluga_estimation::MockMiddleware mock;
  CountingFormatValue::format_count = 0;
  ON_CALL(*mock.logger, IsEnabled(::testing::_)).WillByDefault(Return(false));
  EXPECT_CALL(*mock.logger, Log(::testing::_, ::testing::_)).Times(0);

  mock.Debug("value {}", CountingFormatValue{42});

  EXPECT_EQ(CountingFormatValue::format_count, 0U);
}

TEST(MiddlewareLoggingTest, EnabledFormattedLogFormatsExactlyOnce) {
  beluga_estimation::MockMiddleware mock;
  CountingFormatValue::format_count = 0;
  EXPECT_CALL(*mock.logger, Log(beluga_estimation::LogLevel::kDebug, std::string_view{"value 42"})).Times(1);

  mock.Debug("value {}", CountingFormatValue{42});

  EXPECT_EQ(CountingFormatValue::format_count, 1U);
}

TEST(MiddlewareLoggingTest, RuntimeStringIsLoggedLiterally) {
  beluga_estimation::MockMiddleware mock;
  const std::string message = "literal {braces} and 100%";

  EXPECT_CALL(*mock.logger, Log(beluga_estimation::LogLevel::kDebug, std::string_view{message})).Times(1);

  mock.Debug(message);
}

TEST(MiddlewareLoggingTest, RuntimeStringAsFormatArgumentIsNotInterpreted) {
  beluga_estimation::MockMiddleware mock;
  const std::string payload = "literal {braces}";

  EXPECT_CALL(*mock.logger, Log(beluga_estimation::LogLevel::kDebug, std::string_view{"payload: literal {braces}"}))
      .Times(1);

  mock.Debug("payload: {}", payload);
}

TEST(MiddlewareLoggingTest, ArgumentsAreEvaluatedEvenWhenDisabled) {
  beluga_estimation::MockMiddleware mock;
  ON_CALL(*mock.logger, IsEnabled(::testing::_)).WillByDefault(Return(false));
  EXPECT_CALL(*mock.logger, Log(::testing::_, ::testing::_)).Times(0);

  int evaluations = 0;
  const auto make_argument = [&evaluations] {
    ++evaluations;
    return 42;
  };

  mock.Debug("value {}", make_argument());

  EXPECT_EQ(evaluations, 1);
}

TEST(MiddlewareInterfaceTest, CreateSubscriberForwardsToTopicsInterface) {
  beluga_estimation::MockMiddleware mock;
  auto expected_subscriber = std::make_shared<beluga_estimation::MockSubscriber<MockMessage>>();

  EXPECT_CALL(*mock.topics, CreateSubscriberImpl(Eq(MockMessage::GetStaticId()), Eq("test_topic"), _))
      .WillOnce([expected_subscriber](MessageId, std::string_view, SubscriberCallback<Message>) {
        return std::static_pointer_cast<beluga_estimation::SubscriberBase>(expected_subscriber);
      });

  auto subscriber = mock.CreateSubscriber<MockMessage>("test_topic", [](std::shared_ptr<const MockMessage>) {});
  EXPECT_EQ(subscriber, expected_subscriber);
}

}  // namespace

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

#include <memory>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <beluga_estimation/middleware/message_pipe.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/mocks/mock_message.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>
#include <beluga_estimation/mocks/mock_subscriber.hpp>

using ::testing::_;
using ::testing::Eq;
using ::testing::Return;

namespace {

using beluga_estimation::Message;
using beluga_estimation::MessageId;
using beluga_estimation::MessagePipe;
using beluga_estimation::MockMessage;
using beluga_estimation::MockMiddleware;
using beluga_estimation::MockSubscriber;
using beluga_estimation::SubscriberCallback;

/// @brief Helper that captures the subscriber callback handed to the topics interface.
void CaptureCallback(MockMiddleware& middleware, const char* topic, SubscriberCallback<Message>& callback) {
  EXPECT_CALL(*middleware.topics, CreateSubscriberImpl(Eq(MockMessage::GetStaticId()), Eq(topic), _))
      .WillOnce(testing::DoAll(testing::SaveArg<2>(&callback), testing::InvokeWithoutArgs([]() {
                                 return beluga_estimation::Result_<std::shared_ptr<beluga_estimation::SubscriberBase>>{
                                     std::make_shared<MockSubscriber<MockMessage>>()};
                               })));
}

// =============================================================================
// Lifecycle
// =============================================================================

TEST(MessagePipeTest, IsClosedByDefault) {
  auto middleware = std::make_shared<MockMiddleware>();
  MessagePipe<MockMessage> pipe(middleware, "test_topic");
  EXPECT_FALSE(pipe.IsOpen());
  EXPECT_TRUE(pipe.Empty());
}

TEST(MessagePipeTest, OpenCreatesSubscription) {
  auto middleware = std::make_shared<MockMiddleware>();
  MessagePipe<MockMessage> pipe(middleware, "test_topic");
  EXPECT_CALL(*middleware->topics, CreateSubscriberImpl(Eq(MockMessage::GetStaticId()), Eq("test_topic"), _))
      .WillOnce(testing::InvokeWithoutArgs([]() {
        return beluga_estimation::Result_<std::shared_ptr<beluga_estimation::SubscriberBase>>{
            std::make_shared<MockSubscriber<MockMessage>>()};
      }));

  pipe.Open();

  EXPECT_TRUE(pipe.IsOpen());
}

TEST(MessagePipeTest, OpenIsIdempotent) {
  auto middleware = std::make_shared<MockMiddleware>();
  MessagePipe<MockMessage> pipe(middleware, "test_topic");
  EXPECT_CALL(*middleware->topics, CreateSubscriberImpl(_, _, _)).WillOnce(testing::InvokeWithoutArgs([]() {
    return beluga_estimation::Result_<std::shared_ptr<beluga_estimation::SubscriberBase>>{
        std::make_shared<MockSubscriber<MockMessage>>()};
  }));

  pipe.Open();
  pipe.Open();  // Must not create a second subscription.

  EXPECT_TRUE(pipe.IsOpen());
}

TEST(MessagePipeTest, CloseIsIdempotentAndDiscardsQueuedMessages) {
  auto middleware = std::make_shared<MockMiddleware>();
  MessagePipe<MockMessage> pipe(middleware, "test_topic");
  SubscriberCallback<Message> callback;
  CaptureCallback(*middleware, "test_topic", callback);

  pipe.Open();
  callback(std::make_shared<MockMessage>());
  callback(std::make_shared<MockMessage>());
  EXPECT_FALSE(pipe.Empty());

  pipe.Close();
  EXPECT_FALSE(pipe.IsOpen());
  EXPECT_TRUE(pipe.Empty());

  pipe.Close();  // Idempotent.
}

TEST(MessagePipeTest, DestructorClosesPipe) {
  auto middleware = std::make_shared<MockMiddleware>();
  SubscriberCallback<Message> callback;
  CaptureCallback(*middleware, "test_topic", callback);
  {
    MessagePipe<MockMessage> pipe(middleware, "test_topic");
    pipe.Open();
    callback(std::make_shared<MockMessage>());
  }  // Destructor must close the pipe and discard queued messages.
}

TEST(MessagePipeTest, OpenAfterCloseReopens) {
  auto middleware = std::make_shared<MockMiddleware>();
  MessagePipe<MockMessage> pipe(middleware, "test_topic");
  EXPECT_CALL(*middleware->topics, CreateSubscriberImpl(_, _, _))
      .Times(2)
      .WillRepeatedly(testing::InvokeWithoutArgs([]() {
        return beluga_estimation::Result_<std::shared_ptr<beluga_estimation::SubscriberBase>>{
            std::make_shared<MockSubscriber<MockMessage>>()};
      }));

  pipe.Open();
  pipe.Close();
  pipe.Open();

  EXPECT_TRUE(pipe.IsOpen());
}

// =============================================================================
// Fetching
// =============================================================================

TEST(MessagePipeTest, FetchSingleMessage) {
  auto middleware = std::make_shared<MockMiddleware>();
  MessagePipe<MockMessage> pipe(middleware, "test_topic");
  SubscriberCallback<Message> callback;
  CaptureCallback(*middleware, "test_topic", callback);

  pipe.Open();
  auto message = std::make_shared<MockMessage>();
  callback(message);

  std::shared_ptr<const MockMessage> out;
  EXPECT_EQ(pipe.Fetch(), message);
  EXPECT_EQ(pipe.Fetch(), nullptr);  // Queue is now empty.
}

TEST(MessagePipeTest, FetchAllMessagesReturnsCountInOrder) {
  auto middleware = std::make_shared<MockMiddleware>();
  MessagePipe<MockMessage> pipe(middleware, "test_topic");
  SubscriberCallback<Message> callback;
  CaptureCallback(*middleware, "test_topic", callback);

  pipe.Open();
  auto first = std::make_shared<MockMessage>();
  auto second = std::make_shared<MockMessage>();
  callback(first);
  callback(second);

  std::vector<std::shared_ptr<const MockMessage>> seen;
  auto back_insert = [&seen](std::shared_ptr<const MockMessage> message) { seen.push_back(std::move(message)); };
  const std::size_t count = pipe.Fetch(back_insert);

  EXPECT_EQ(count, 2u);
  ASSERT_EQ(seen.size(), 2u);
  EXPECT_EQ(seen[0], first);
  EXPECT_EQ(seen[1], second);
  EXPECT_TRUE(pipe.Empty());
}

TEST(MessagePipeTest, FetchOnClosedPipeIsEmpty) {
  auto middleware = std::make_shared<MockMiddleware>();
  MessagePipe<MockMessage> pipe(middleware, "test_topic");

  std::shared_ptr<const MockMessage> out;
  EXPECT_EQ(pipe.Fetch(), nullptr);  // Queue is now empty.
  EXPECT_EQ(pipe.Fetch([](std::shared_ptr<const MockMessage>) {}), 0u);
}

TEST(MessagePipeTest, FetchWithThrowingCallbackPropagatesAndKeepsRemainder) {
  auto middleware = std::make_shared<MockMiddleware>();
  MessagePipe<MockMessage> pipe(middleware, "test_topic");
  SubscriberCallback<Message> callback;
  CaptureCallback(*middleware, "test_topic", callback);

  pipe.Open();
  callback(std::make_shared<MockMessage>());
  callback(std::make_shared<MockMessage>());

  int seen = 0;
  EXPECT_THROW(
      pipe.Fetch([&seen](std::shared_ptr<const MockMessage>) {
        ++seen;
        throw std::runtime_error("boom");
      }),
      std::runtime_error);
  EXPECT_EQ(seen, 1);          // First message was processed before the throw.
  EXPECT_FALSE(pipe.Empty());  // Second message remains queued.

  std::shared_ptr<const MockMessage> out;
  EXPECT_NE(pipe.Fetch(), nullptr);
  EXPECT_EQ(pipe.Fetch(), nullptr);  // Queue now empty.
}

TEST(MessagePipeTest, FetchWorksAfterReopen) {
  auto middleware = std::make_shared<MockMiddleware>();
  MessagePipe<MockMessage> pipe(middleware, "test_topic");

  SubscriberCallback<Message> first_callback;

  CaptureCallback(*middleware, "test_topic", first_callback);
  pipe.Open();
  first_callback(std::make_shared<MockMessage>());  // Discarded on close.
  pipe.Close();

  SubscriberCallback<Message> second_callback;

  CaptureCallback(*middleware, "test_topic", second_callback);
  pipe.Open();
  auto message = std::make_shared<MockMessage>();
  second_callback(message);

  EXPECT_EQ(pipe.Fetch(), message);
  EXPECT_EQ(pipe.Fetch(), nullptr);
}

}  // namespace

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

#include <gmock/gmock.h>

#include <beluga_estimation/middleware/topics_interface.hpp>
#include <beluga_estimation/mocks/mock_message.hpp>
#include <beluga_estimation/mocks/mock_publisher.hpp>
#include <beluga_estimation/mocks/mock_subscriber.hpp>
#include <beluga_estimation/mocks/mock_topics.hpp>

namespace {

using beluga_estimation::Message;
using beluga_estimation::MessageId;
using beluga_estimation::MockMessage;
using beluga_estimation::SubscriberCallback;
using testing::_;
using testing::Eq;
using testing::Return;

/// A message type distinct from `MockMessage`, for mismatch tests.
class OtherMessage : public Message {
 public:
  static MessageId GetStaticId() { return beluga_estimation::HashStringView("OtherMessage"); }

  MessageId GetId() const final { return GetStaticId(); }

  const void* GetNativeHandle() const final { return nullptr; }
};

TEST(TopicsInterfaceTest, CreatePublisher) {
  beluga_estimation::MockTopics mock;
  auto expected_publisher = std::make_shared<beluga_estimation::MockPublisher<MockMessage>>();

  EXPECT_CALL(mock, CreatePublisherImpl(Eq(MockMessage::GetStaticId()), Eq("test_topic")))
      .WillOnce(Return(expected_publisher));

  auto publisher = mock.CreatePublisher<MockMessage>("test_topic");
  EXPECT_EQ(publisher, expected_publisher);
}

TEST(TopicsInterfaceTest, CreateSubscriber) {
  beluga_estimation::MockTopics mock;
  auto expected_subscriber = std::make_shared<beluga_estimation::MockSubscriber<MockMessage>>();

  EXPECT_CALL(mock, CreateSubscriberImpl(Eq(MockMessage::GetStaticId()), Eq("test_topic"), _))
      .WillOnce([expected_subscriber](MessageId, std::string_view, SubscriberCallback<Message>) {
        return std::static_pointer_cast<beluga_estimation::SubscriberBase>(expected_subscriber);
      });

  auto subscriber = mock.CreateSubscriber<MockMessage>("test_topic", [](std::shared_ptr<const MockMessage>) {});
  EXPECT_EQ(subscriber, expected_subscriber);
}

TEST(TopicsInterfaceTest, PublishRoutesToPublishImpl) {
  auto mock_publisher = std::make_shared<beluga_estimation::MockPublisher<MockMessage>>();
  MockMessage message;

  EXPECT_CALL(*mock_publisher, PublishImpl(testing::Ref(message)));

  mock_publisher->Publish(message);
}

TEST(TopicsInterfaceTest, SubscriberCallbackCastsAndDelivers) {
  beluga_estimation::MockTopics mock;
  auto expected_subscriber = std::make_shared<beluga_estimation::MockSubscriber<MockMessage>>();

  SubscriberCallback<Message> captured_callback;
  EXPECT_CALL(mock, CreateSubscriberImpl(Eq(MockMessage::GetStaticId()), Eq("test_topic"), _))
      .WillOnce([&](MessageId, std::string_view, SubscriberCallback<Message> callback) {
        captured_callback = std::move(callback);
        return std::static_pointer_cast<beluga_estimation::SubscriberBase>(expected_subscriber);
      });

  bool callback_called = false;
  mock.CreateSubscriber<MockMessage>("test_topic", [&](std::shared_ptr<const MockMessage>) { callback_called = true; });

  captured_callback(std::make_shared<MockMessage>());
  EXPECT_TRUE(callback_called);
}

TEST(TopicsInterfaceTest, SubscriberCallbackRejectsWrongMessageType) {
  beluga_estimation::MockTopics mock;
  auto expected_subscriber = std::make_shared<beluga_estimation::MockSubscriber<MockMessage>>();

  SubscriberCallback<Message> captured_callback;
  EXPECT_CALL(mock, CreateSubscriberImpl(Eq(MockMessage::GetStaticId()), Eq("test_topic"), _))
      .WillOnce([&](MessageId, std::string_view, SubscriberCallback<Message> callback) {
        captured_callback = std::move(callback);
        return std::static_pointer_cast<beluga_estimation::SubscriberBase>(expected_subscriber);
      });

  bool callback_called = false;
  mock.CreateSubscriber<MockMessage>("test_topic", [&](std::shared_ptr<const MockMessage>) { callback_called = true; });

  EXPECT_THROW(captured_callback(std::make_shared<OtherMessage>()), tl::bad_expected_access<beluga_estimation::Error>);
  EXPECT_FALSE(callback_called);
}

TEST(TopicsInterfaceTest, SubscriberCallbackRejectsNullMessage) {
  beluga_estimation::MockTopics mock;
  auto expected_subscriber = std::make_shared<beluga_estimation::MockSubscriber<MockMessage>>();

  SubscriberCallback<Message> captured_callback;
  EXPECT_CALL(mock, CreateSubscriberImpl(Eq(MockMessage::GetStaticId()), Eq("test_topic"), _))
      .WillOnce([&](MessageId, std::string_view, SubscriberCallback<Message> callback) {
        captured_callback = std::move(callback);
        return std::static_pointer_cast<beluga_estimation::SubscriberBase>(expected_subscriber);
      });

  bool callback_called = false;
  mock.CreateSubscriber<MockMessage>("test_topic", [&](std::shared_ptr<const MockMessage>) { callback_called = true; });

  EXPECT_THROW(captured_callback(nullptr), std::runtime_error);
  EXPECT_FALSE(callback_called);
}

}  // namespace

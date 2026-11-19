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

#include <gmock/gmock.h>

#include <rclcpp/rclcpp.hpp>

#include <beluga_estimation_ros/converters/header.hpp>
#include <beluga_estimation_ros/interfaces/message.hpp>
#include <beluga_estimation_ros/interfaces/topics.hpp>

namespace {

using ::testing::Eq;
using ::testing::MockFunction;
using ::testing::Pointee;
using ::testing::Property;

class TopicsInterfaceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node", rclcpp::NodeOptions().use_intra_process_comms(true));
    topics_ = std::make_unique<beluga_estimation_ros::Topics>(
        node_->get_node_parameters_interface(), node_->get_node_topics_interface());
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<beluga_estimation_ros::Topics> topics_;
};

TEST_F(TopicsInterfaceTest, PubSubRoundTrip) {
  beluga_estimation_ros::msg::MutableHeader message;
  message.SetFrameId("test_frame");

  MockFunction<void(std::shared_ptr<const beluga_estimation::msg::Header>)> callback;
  EXPECT_CALL(callback, Call(Pointee(Property(&beluga_estimation::msg::Header::GetFrameId, Eq("test_frame")))));

  auto publisher = topics_->CreatePublisher<beluga_estimation::msg::Header>("test_topic");
  auto subscriber = topics_->CreateSubscriber<beluga_estimation::msg::Header>("test_topic", callback.AsStdFunction());
  ASSERT_TRUE(publisher);
  ASSERT_TRUE(subscriber);

  (*publisher)->Publish(message);

  rclcpp::spin_some(node_);
}

TEST_F(TopicsInterfaceTest, PublisherGetTopicName) {
  auto publisher = topics_->CreatePublisher<beluga_estimation::msg::Header>("my_topic");
  ASSERT_TRUE(publisher);
  EXPECT_THAT((*publisher)->GetTopicName(), testing::HasSubstr("my_topic"));
}

TEST_F(TopicsInterfaceTest, SubscriberGetTopicName) {
  auto subscriber = topics_->CreateSubscriber<beluga_estimation::msg::Header>(
      "my_topic", [](std::shared_ptr<const beluga_estimation::msg::Header>) {});
  ASSERT_TRUE(subscriber);
  EXPECT_THAT((*subscriber)->GetTopicName(), testing::HasSubstr("my_topic"));
}

TEST_F(TopicsInterfaceTest, PublisherGetSubscriptionCount) {
  auto publisher = topics_->CreatePublisher<beluga_estimation::msg::Header>("count_topic");
  ASSERT_TRUE(publisher);
  EXPECT_EQ((*publisher)->GetSubscriptionCount(), 0U);
}

TEST_F(TopicsInterfaceTest, SubscriberGetPublisherCount) {
  auto subscriber = topics_->CreateSubscriber<beluga_estimation::msg::Header>(
      "count_topic", [](std::shared_ptr<const beluga_estimation::msg::Header>) {});
  ASSERT_TRUE(subscriber);
  EXPECT_EQ((*subscriber)->GetPublisherCount(), 0U);
}

TEST_F(TopicsInterfaceTest, UnregisteredPublisherTypeReturnsError) {
  constexpr beluga_estimation::MessageId kUnregisteredId = 0;
  auto factory = beluga_estimation_ros::MessageFactoryRegistry::GetPublisherFactory(kUnregisteredId);
  EXPECT_FALSE(factory);
  EXPECT_EQ(factory.error(), beluga_estimation::Error::kFailure);
}

TEST_F(TopicsInterfaceTest, UnregisteredSubscriberTypeReturnsError) {
  constexpr beluga_estimation::MessageId kUnregisteredId = 0;
  auto factory = beluga_estimation_ros::MessageFactoryRegistry::GetSubscriberFactory(kUnregisteredId);
  EXPECT_FALSE(factory);
  EXPECT_EQ(factory.error(), beluga_estimation::Error::kFailure);
}

TEST_F(TopicsInterfaceTest, PublisherDefaultQoS) {
  auto publisher = std::make_shared<beluga_estimation_ros::Publisher<beluga_estimation_ros::msg::Header>>(
      node_->get_node_parameters_interface(), node_->get_node_topics_interface(), "pub_default_topic");

  auto info = node_->get_publishers_info_by_topic("/pub_default_topic");
  ASSERT_EQ(info.size(), 1U);
  EXPECT_EQ(info[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(info[0].qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST_F(TopicsInterfaceTest, PublisherTransientLocalQoS) {
  auto publisher = std::make_shared<beluga_estimation_ros::Publisher<beluga_estimation_ros::msg::Header>>(
      node_->get_node_parameters_interface(), node_->get_node_topics_interface(), "pub_latched_topic", true);

  auto info = node_->get_publishers_info_by_topic("/pub_latched_topic");
  ASSERT_EQ(info.size(), 1U);
  EXPECT_EQ(info[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(info[0].qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

TEST_F(TopicsInterfaceTest, SubscriberDefaultQoS) {
  auto subscriber = std::make_shared<beluga_estimation_ros::Subscriber<beluga_estimation_ros::msg::Header>>(
      node_->get_node_parameters_interface(), node_->get_node_topics_interface(), "sub_default_topic",
      [](std::shared_ptr<const beluga_estimation::Message>) {});

  auto info = node_->get_subscriptions_info_by_topic("/sub_default_topic");
  ASSERT_EQ(info.size(), 1U);
  EXPECT_EQ(info[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(info[0].qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST_F(TopicsInterfaceTest, SubscriberTransientLocalQoS) {
  auto subscriber = std::make_shared<beluga_estimation_ros::Subscriber<beluga_estimation_ros::msg::Header>>(
      node_->get_node_parameters_interface(), node_->get_node_topics_interface(), "sub_latched_topic",
      [](std::shared_ptr<const beluga_estimation::Message>) {}, true);

  auto info = node_->get_subscriptions_info_by_topic("/sub_latched_topic");
  ASSERT_EQ(info.size(), 1U);
  EXPECT_EQ(info[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(info[0].qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

}  // namespace

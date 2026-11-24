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

#include <memory>
#include <string_view>

#include <rclcpp/create_subscription.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>

#include <beluga_estimation/common/message.hpp>
#include <beluga_estimation/middleware/subscriber.hpp>

namespace beluga_estimation_ros {

/**
 * @brief ROS 2 implementation of the Beluga subscriber interface.
 *
 * Wraps an rclcpp subscription to deliver messages through the
 * middleware-agnostic @ref beluga_estimation::Subscriber interface.
 *
 * @tparam T The Beluga ROS message wrapper type (must expose `BaseType` and `RosType`).
 * @tparam B The middleware-agnostic base message type (defaults to `T::BaseType`).
 */
template <typename T, typename B = typename T::BaseType>
class Subscriber : public beluga_estimation::Subscriber<B> {
 public:
  /**
   * @brief Constructs a Subscriber instance.
   * @param node_parameters The ROS 2 node parameters interface.
   * @param node_topics The ROS 2 node topics interface.
   * @param topic The topic name to subscribe to.
   * @param callback The function to invoke when a message is received.
   * @param transient_local If true, requests transient-local durability to receive the last retained message on
   * connect.
   */
  Subscriber(
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
      std::string_view topic,
      beluga_estimation::SubscriberCallback<beluga_estimation::Message> callback,
      bool transient_local = false)
      : subscriber_(rclcpp::create_subscription<U>(
            node_parameters,
            node_topics,
            std::string(topic),
            transient_local ? rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local() : rclcpp::SensorDataQoS(),
            [callback = std::move(callback)](std::shared_ptr<const U> message) {
              callback(std::make_shared<T>(std::move(message)));
            })) {}

  /// \copydoc beluga_estimation::SubscriberBase::GetTopicName
  std::string_view GetTopicName() const final { return subscriber_->get_topic_name(); }

  /// \copydoc beluga_estimation::SubscriberBase::GetPublisherCount
  std::size_t GetPublisherCount() const final { return subscriber_->get_publisher_count(); }

 private:
  using U = typename T::RosType;

  typename rclcpp::Subscription<U>::SharedPtr subscriber_;
};

}  // namespace beluga_estimation_ros

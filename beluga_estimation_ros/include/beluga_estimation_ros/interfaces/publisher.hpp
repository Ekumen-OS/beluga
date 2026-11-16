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

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>

#include <beluga_estimation/middleware/publisher.hpp>
#include <beluga_estimation_ros/converters/fwd.hpp>

namespace beluga_estimation_ros {

/**
 * @brief ROS 2 implementation of the Beluga publisher interface.
 *
 * Wraps an rclcpp publisher to expose message publishing through the
 * middleware-agnostic @ref beluga_estimation::Publisher interface.
 *
 * @tparam T The Beluga ROS message wrapper type (must expose `BaseType` and `RosType`).
 * @tparam B The middleware-agnostic base message type (defaults to `T::BaseType`).
 */
template <typename T, typename B = typename T::BaseType>
class Publisher : public beluga_estimation::Publisher<B> {
  using U = typename T::RosType;

 public:
  /**
   * @brief Constructs a Publisher instance.
   * @param node_parameters The ROS 2 node parameters interface.
   * @param node_topics The ROS 2 node topics interface.
   * @param topic The topic name to publish to.
   * @param transient_local If true, retains the last message for late-joining subscribers (transient-local durability).
   */
  Publisher(
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
      std::string_view topic,
      bool transient_local = false)
      : publisher_(rclcpp::create_publisher<U>(
            node_parameters,
            node_topics,
            std::string(topic),
            transient_local ? rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local() : rclcpp::QoS(10))) {}

  /// \copydoc beluga_estimation::PublisherBase::GetTopicName
  std::string_view GetTopicName() const final { return publisher_->get_topic_name(); }

  /// \copydoc beluga_estimation::PublisherBase::GetSubscriptionCount
  std::size_t GetSubscriptionCount() const final { return publisher_->get_subscription_count(); }

 protected:
  /// \copydoc beluga_estimation::Publisher::PublishImpl
  void PublishImpl(const B& message) final { publisher_->publish(ToRos(message)); }

 private:
  typename rclcpp::Publisher<U>::SharedPtr publisher_;
};

}  // namespace beluga_estimation_ros

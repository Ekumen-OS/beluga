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
 * @brief Includes ROS 2 message registration and factory utilities.
 */

#include <stdexcept>
#include <unordered_map>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation_ros/interfaces/publisher.hpp>
#include <beluga_estimation_ros/interfaces/subscriber.hpp>

namespace beluga_estimation_ros {

/**
 * @brief Registry for ROS 2 message publishers and subscribers.
 *
 * This class uses an indexed dispatcher to map message IDs to factory functions
 * that create ROS 2-specific publishers and subscribers.
 */
class MessageFactoryRegistry {
 public:
  /// Type alias for a publisher factory function.
  using PublisherFactory = std::function<std::shared_ptr<beluga_estimation::PublisherBase>(
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr,
      std::string_view)>;

  /// Type alias for a subscriber factory function.
  using SubscriberFactory = std::function<std::shared_ptr<beluga_estimation::SubscriberBase>(
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr,
      std::string_view,
      beluga_estimation::SubscriberCallback<beluga_estimation::Message>)>;

  /// Map of message IDs to publisher factories.
  using PublisherFactoryRegistry = std::unordered_map<beluga_estimation::MessageId, PublisherFactory>;

  /// Map of message IDs to subscriber factories.
  using SubscriberFactoryRegistry = std::unordered_map<beluga_estimation::MessageId, SubscriberFactory>;

  /// Gets the singleton instance of the registry.
  static MessageFactoryRegistry& Instance() {
    static MessageFactoryRegistry instance;
    return instance;
  }

  /**
   * @brief Gets the publisher factory for a given message type.
   * @param type The message ID.
   * @return A pointer to the factory, or an error if none is registered for @p type.
   */
  static beluga_estimation::Result_<const PublisherFactory*> GetPublisherFactory(beluga_estimation::MessageId type) {
    auto& registry = Instance();
    auto it = registry.publisher_factory_registry_.find(type);
    if (it == registry.publisher_factory_registry_.end()) {
      return beluga_estimation::Err(
          beluga_estimation::Error::kFailure,
          fmt::format("no publisher factory registered for message type id {}", type));
    }
    return &it->second;
  }

  /**
   * @brief Gets the subscriber factory for a given message type.
   * @param type The message ID.
   * @return A pointer to the factory, or an error if none is registered for @p type.
   */
  static beluga_estimation::Result_<const SubscriberFactory*> GetSubscriberFactory(beluga_estimation::MessageId type) {
    auto& registry = Instance();
    auto it = registry.subscriber_factory_registry_.find(type);
    if (it == registry.subscriber_factory_registry_.end()) {
      return beluga_estimation::Err(
          beluga_estimation::Error::kFailure,
          fmt::format("no subscriber factory registered for message type id {}", type));
    }
    return &it->second;
  }

  /**
   * @brief Registers a message type with the factory.
   *
   * @tparam T The ROS 2 message type to register.
   * @param transient_local If true, applies transient-local durability: publishers retain the last message
   *                        and subscribers request it on connect.
   */
  template <typename T>
  static void RegisterMessage(bool transient_local = false) {
    static const bool registered_as = transient_local;
    [[maybe_unused]] static bool once = [transient_local]() {
      Instance().publisher_factory_registry_.insert(
          {T::GetStaticId(),  //
           [transient_local](
               rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,  //
               rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,          //
               std::string_view topic) {                                                     //
             return std::make_shared<beluga_estimation_ros::Publisher<T>>(                   //
                 node_parameters,                                                            //
                 node_topics,                                                                //
                 topic,                                                                      //
                 transient_local);
           }});

      Instance().subscriber_factory_registry_.insert(
          {T::GetStaticId(),  //
           [transient_local](
               rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,   //
               rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,           //
               std::string_view topic,                                                        //
               beluga_estimation::SubscriberCallback<beluga_estimation::Message> callback) {  //
             return std::make_shared<beluga_estimation_ros::Subscriber<T>>(                   //
                 node_parameters,                                                             //
                 node_topics,                                                                 //
                 topic,                                                                       //
                 std::move(callback),                                                         //
                 transient_local);
           }});

      return true;
    }();

    BELUGA_ESTIMATION_DEMAND(
        transient_local == registered_as,
        "conflicting registration for message type id {}: already registered with transient_local={}, cannot "
        "re-register with transient_local={}",
        T::GetStaticId(), registered_as, transient_local);
  }

 private:
  MessageFactoryRegistry() = default;
  ~MessageFactoryRegistry() = default;
  MessageFactoryRegistry(const MessageFactoryRegistry&) = delete;
  MessageFactoryRegistry& operator=(const MessageFactoryRegistry&) = delete;
  MessageFactoryRegistry(MessageFactoryRegistry&&) = delete;
  MessageFactoryRegistry& operator=(MessageFactoryRegistry&&) = delete;

  PublisherFactoryRegistry publisher_factory_registry_;
  SubscriberFactoryRegistry subscriber_factory_registry_;
};

}  // namespace beluga_estimation_ros

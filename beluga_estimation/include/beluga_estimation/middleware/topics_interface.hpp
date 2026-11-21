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
 * @brief Includes topics interface declarations.
 */

#include <memory>
#include <string>
#include <string_view>
#include <typeinfo>

#include <beluga_estimation/common/message.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/type_casts.hpp>
#include <beluga_estimation/middleware/publisher.hpp>
#include <beluga_estimation/middleware/subscriber.hpp>

namespace beluga_estimation {

/**
 * @brief Interface for managing publishers and subscribers.
 *
 * This interface provides a type-safe way to create publishers and subscribers
 * for different message types over a middleware.
 */
class TopicsInterface {
 public:
  virtual ~TopicsInterface() = default;

  /**
   * @brief Creates a publisher for a specific message type.
   * @tparam T The type of the message to publish.
   * @param topic The name of the topic to publish to.
   * @return The created publisher, or an error if the message type is not available.
   */
  template <typename T>
  Result_<std::shared_ptr<Publisher<T>>> CreatePublisher(std::string_view topic) {
    auto result = CreatePublisherImpl(T::GetStaticId(), topic);
    if (!result) {
      return Err(CollectError(result));
    }
    return safe_dynamic_pointer_cast<Publisher<T>>(Unwrap(result), "publisher", topic);
  }

  /**
   * @brief Creates a subscriber for a specific message type.
   * @tparam T The type of the message to subscribe to.
   * @param topic The name of the topic to subscribe to.
   * @param callback The function to call when a message is received.
   * @return The created subscriber, or an error if the message type is not available.
   */
  template <typename T>
  Result_<std::shared_ptr<Subscriber<T>>> CreateSubscriber(std::string_view topic, SubscriberCallback<T> callback) {
    auto result = CreateSubscriberImpl(
        T::GetStaticId(), topic, [callback = std::move(callback)](std::shared_ptr<const Message> message) {
          auto typed_message =
              Unwrap(safe_dynamic_pointer_cast<const T>(std::move(message), "message", typeid(T).name()));
          BELUGA_ESTIMATION_DEMAND(typed_message != nullptr, "received a null message");
          callback(std::move(typed_message));
        });
    if (!result) {
      return Err(CollectError(result));
    }
    return safe_dynamic_pointer_cast<Subscriber<T>>(Unwrap(result), "subscriber", topic);
  }

 protected:
  virtual Result_<std::shared_ptr<PublisherBase>> CreatePublisherImpl(  //
      MessageId type,                                                   //
      std::string_view topic) = 0;

  virtual Result_<std::shared_ptr<SubscriberBase>> CreateSubscriberImpl(  //
      MessageId type,                                                     //
      std::string_view topic,
      SubscriberCallback<Message> callback) = 0;
};

}  // namespace beluga_estimation

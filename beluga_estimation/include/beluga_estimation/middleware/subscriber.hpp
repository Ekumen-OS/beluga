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
 * @brief Includes subscriber declarations.
 */

#include <cstddef>
#include <functional>
#include <memory>
#include <string_view>

#include <beluga_estimation/common/message.hpp>

namespace beluga_estimation {

/**
 * @brief Type alias for a subscriber callback function.
 * @tparam T The type of the message received.
 */
template <typename T>
using SubscriberCallback = std::function<void(std::shared_ptr<const T>)>;

/**
 * @brief Base class for all subscribers.
 */
class SubscriberBase {
 public:
  virtual ~SubscriberBase() = default;

  /// Returns the name of the topic this subscriber is subscribed to.
  virtual std::string_view GetTopicName() const = 0;

  /// Returns the number of publishers currently publishing to this topic.
  virtual std::size_t GetPublisherCount() const = 0;
};

/**
 * @brief A subscriber for a specific message type.
 * @tparam T The type of the message to subscribe to.
 */
template <typename T>
class Subscriber : public SubscriberBase {};

}  // namespace beluga_estimation

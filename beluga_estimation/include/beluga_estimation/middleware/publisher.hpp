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
 * @brief Includes publisher declarations.
 */

#include <cstddef>
#include <string_view>
#include <type_traits>

#include <beluga_estimation/common/message.hpp>

namespace beluga_estimation {

/**
 * @brief Base class for all publishers.
 */
class PublisherBase {
 public:
  virtual ~PublisherBase() = default;

  /// Returns the name of the topic this publisher is publishing to.
  virtual std::string_view GetTopicName() const = 0;

  /// Returns the number of subscribers currently listening to this topic.
  virtual std::size_t GetSubscriptionCount() const = 0;
};

/**
 * @brief A publisher for a specific message type.
 * @tparam T The type of the message to publish.
 */
template <typename T>
class Publisher : public PublisherBase {
 public:
  static_assert(
      std::is_abstract_v<T> || sizeof(T) == sizeof(beluga_estimation::Message),
      "Message type T must either be abstract or have no additional data members "
      "to prevent object slicing.");

  /// Publishes a message to the topic.
  void Publish(const T& message) { PublishImpl(message); }

 protected:
  /// Implementation hook called by Publish(). Derived classes provide the middleware-specific delivery.
  virtual void PublishImpl(const T& message) = 0;
};

}  // namespace beluga_estimation

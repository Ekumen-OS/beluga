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
 * @brief Includes topics interface mock.
 */

#include <gmock/gmock.h>

#include <beluga_estimation/middleware/topics_interface.hpp>

namespace beluga_estimation {

class MockTopics : public TopicsInterface {
 public:
  using TopicsInterface::CreatePublisher;
  using TopicsInterface::CreateSubscriber;

  MOCK_METHOD(
      (Result_<std::shared_ptr<PublisherBase>>),
      CreatePublisherImpl,
      (MessageId, std::string_view),
      (override));

  MOCK_METHOD(
      (Result_<std::shared_ptr<SubscriberBase>>),
      CreateSubscriberImpl,
      (MessageId, std::string_view, SubscriberCallback<Message>),
      (override));
};

}  // namespace beluga_estimation

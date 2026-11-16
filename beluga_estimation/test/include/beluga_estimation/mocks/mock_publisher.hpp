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
 * @brief Includes publisher mock.
 */

#include <gmock/gmock.h>

#include <beluga_estimation/middleware/publisher.hpp>

namespace beluga_estimation {

template <typename T>
class MockPublisher : public Publisher<T> {
 public:
  MOCK_METHOD(std::string_view, GetTopicName, (), (const, override));
  MOCK_METHOD(std::size_t, GetSubscriptionCount, (), (const, override));
  MOCK_METHOD(void, PublishImpl, (const T& message), (override));
};

}  // namespace beluga_estimation

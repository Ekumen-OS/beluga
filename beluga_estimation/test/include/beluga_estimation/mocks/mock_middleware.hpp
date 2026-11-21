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
 * @brief Includes middleware interface mock.
 */

#include <gmock/gmock.h>

#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/mocks/mock_logger.hpp>
#include <beluga_estimation/mocks/mock_parameters.hpp>
#include <beluga_estimation/mocks/mock_topics.hpp>
#include <beluga_estimation/mocks/mock_transform.hpp>

namespace beluga_estimation {

class MockMiddleware : public MiddlewareInterface {
 public:
  using MiddlewareInterface::CreatePublisher;
  using MiddlewareInterface::CreateSubscriber;
  using MiddlewareInterface::DeclareParameter;
  using MiddlewareInterface::GetParameter;

  MOCK_METHOD(std::string_view, GetBackendName, (), (override, const));

  MOCK_METHOD(void*, GetBackendObject, (), (override));

  ParametersInterface* GetParametersInterface() override { return parameters.get(); }
  TopicsInterface* GetTopicsInterface() override { return topics.get(); }
  TransformInterface* GetTransformInterface() override { return transform.get(); }
  LoggerInterface* GetLoggerInterface() override { return logger.get(); }

  MOCK_METHOD(ClockInterface*, GetClockInterface, (), (override));
  MOCK_METHOD(std::unique_ptr<RateInterface>, CreateRate, (double frequency), (override));

  std::unique_ptr<MockParameters> parameters = std::make_unique<MockParameters>();
  std::unique_ptr<MockTopics> topics = std::make_unique<MockTopics>();
  std::unique_ptr<MockTransform> transform = std::make_unique<MockTransform>();
  std::unique_ptr<MockLogger> logger = std::make_unique<MockLogger>();
};

}  // namespace beluga_estimation

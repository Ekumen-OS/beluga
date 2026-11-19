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

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation_ros/interfaces/middleware.hpp>

namespace beluga_estimation_ros {

std::shared_ptr<NodeInterface> GetRosNodeInterface(std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware) {
  BELUGA_ESTIMATION_DEMAND(middleware->GetBackendName() == "ros", "unsupported backend type, expected ROS");
  return {middleware, reinterpret_cast<NodeInterface*>(middleware->GetBackendObject())};
}

Middleware::Middleware(std::shared_ptr<NodeInterface> node_interface)
    : node_(std::move(node_interface)),
      parameters_(std::make_unique<beluga_estimation_ros::Parameters>(node_->get_node_parameters_interface())),
      topics_(std::make_unique<beluga_estimation_ros::Topics>(
          node_->get_node_parameters_interface(),
          node_->get_node_topics_interface())),
      clock_(std::make_unique<beluga_estimation_ros::Clock>(node_->get_node_clock_interface()->get_clock())),
      transform_(std::make_unique<beluga_estimation_ros::TF2Interface>(node_)),
      logger_(std::make_unique<beluga_estimation_ros::Logger>(node_->get_node_logging_interface())) {}

std::string_view Middleware::GetBackendName() const {
  return "ros";
}

void* Middleware::GetBackendObject() {
  return node_.get();
}

beluga_estimation::ParametersInterface* Middleware::GetParametersInterface() {
  return parameters_.get();
}

beluga_estimation::TopicsInterface* Middleware::GetTopicsInterface() {
  return topics_.get();
}

beluga_estimation::ClockInterface* Middleware::GetClockInterface() {
  return clock_.get();
}

beluga_estimation::TransformInterface* Middleware::GetTransformInterface() {
  return transform_.get();
}

beluga_estimation::LoggerInterface* Middleware::GetLoggerInterface() {
  return logger_.get();
}

std::unique_ptr<beluga_estimation::RateInterface> Middleware::CreateRate(double frequency) {
  return std::make_unique<beluga_estimation_ros::Rate>(frequency, node_->get_node_clock_interface()->get_clock());
}

}  // namespace beluga_estimation_ros

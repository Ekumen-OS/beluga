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

#include <rclcpp/node_interfaces/node_topics_interface.hpp>

#include <beluga_estimation/middleware/topics_interface.hpp>

namespace beluga_estimation_ros {

class Topics : public beluga_estimation::TopicsInterface {
 public:
  Topics(
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
      rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics);

 private:
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;

  beluga_estimation::Result_<std::shared_ptr<beluga_estimation::PublisherBase>> CreatePublisherImpl(  //
      beluga_estimation::MessageId type,                                                              //
      std::string_view topic) final;

  beluga_estimation::Result_<std::shared_ptr<beluga_estimation::SubscriberBase>> CreateSubscriberImpl(  //
      beluga_estimation::MessageId type,                                                                //
      std::string_view topic,
      beluga_estimation::SubscriberCallback<beluga_estimation::Message> callback) final;
};

}  // namespace beluga_estimation_ros

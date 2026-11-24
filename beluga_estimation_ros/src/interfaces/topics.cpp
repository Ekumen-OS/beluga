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

#include <beluga_estimation_ros/converters/header.hpp>
#include <beluga_estimation_ros/converters/imu.hpp>
#include <beluga_estimation_ros/converters/odometry.hpp>
#include <beluga_estimation_ros/interfaces/message.hpp>
#include <beluga_estimation_ros/interfaces/topics.hpp>

namespace beluga_estimation_ros {

Topics::Topics(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics)
    : node_parameters_(node_parameters), node_topics_(node_topics) {
  MessageFactoryRegistry::RegisterMessage<beluga_estimation_ros::msg::Header>();
  MessageFactoryRegistry::RegisterMessage<beluga_estimation_ros::msg::Imu>();
  MessageFactoryRegistry::RegisterMessage<beluga_estimation_ros::msg::Odometry>();
}

beluga_estimation::Result_<std::shared_ptr<beluga_estimation::PublisherBase>> Topics::CreatePublisherImpl(  //
    beluga_estimation::MessageId type,                                                                      //
    std::string_view topic) {                                                                               //
  auto factory = MessageFactoryRegistry::GetPublisherFactory(type);
  if (!factory) {
    return beluga_estimation::Err(beluga_estimation::CollectError(factory));
  }
  return (**factory)(node_parameters_, node_topics_, topic);
}

beluga_estimation::Result_<std::shared_ptr<beluga_estimation::SubscriberBase>> Topics::CreateSubscriberImpl(  //
    beluga_estimation::MessageId type,                                                                        //
    std::string_view topic,                                                                                   //
    beluga_estimation::SubscriberCallback<beluga_estimation::Message> callback) {                             //
  auto factory = MessageFactoryRegistry::GetSubscriberFactory(type);
  if (!factory) {
    return beluga_estimation::Err(beluga_estimation::CollectError(factory));
  }
  return (**factory)(node_parameters_, node_topics_, topic, callback);
}

}  // namespace beluga_estimation_ros

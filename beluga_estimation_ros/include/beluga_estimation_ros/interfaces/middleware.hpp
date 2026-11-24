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

#include <memory>

#include <rclcpp/node_interfaces/node_interfaces.hpp>

#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation_ros/interfaces/clock.hpp>
#include <beluga_estimation_ros/interfaces/logger.hpp>
#include <beluga_estimation_ros/interfaces/parameters.hpp>
#include <beluga_estimation_ros/interfaces/rate.hpp>
#include <beluga_estimation_ros/interfaces/topics.hpp>
#include <beluga_estimation_ros/interfaces/transform.hpp>

/**
 * @file
 * @brief Includes ROS 2 middleware implementation.
 */

namespace beluga_estimation_ros {

/**
 * @brief A complete ROS 2 node interface type.
 *
 * Node interfaces are the common denominator among node-like objects in rclcpp, such as
 * rclcpp::Node and rclcpp_lifecycle::LifecycleNode.
 */
using NodeInterface = rclcpp::node_interfaces::NodeInterfaces<ALL_RCLCPP_NODE_INTERFACES>;

/**
 * @brief Extracts the underlying ROS 2 node interface from a ROS 2 node like object.
 *
 * @param node The ROS 2 node to extract the interface from.
 * @return A shared pointer to the underlying ROS 2 node interface.
 */
template <class Node>
std::shared_ptr<NodeInterface> MakeNodeInterface(Node&& node) {
  if constexpr (rcpputils::is_pointer<Node>::value) {
    return std::make_shared<NodeInterface>(*node);
  } else {
    return std::make_shared<NodeInterface>(node);
  }
}

/**
 * @brief Extracts the underlying ROS 2 node interface from a middleware interface.
 *
 * @param middleware The middleware interface instance.
 * @return A shared pointer to the underlying ROS 2 node interface.
 * @throws std::runtime_error If the middleware backend is not ROS 2.
 */
std::shared_ptr<NodeInterface> GetRosNodeInterface(std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware);

/**
 * @brief ROS 2 implementation of the Beluga middleware interface.
 *
 * This class wraps an rclcpp::Node to provide parameter and topic management
 * using ROS 2 as the backend.
 */

class Middleware : public beluga_estimation::MiddlewareInterface {
 public:
  /**
   * @brief Constructs a Middleware instance.
   * @param node The ROS 2 node to wrap.
   */

  template <class Node>
  explicit Middleware(Node&& node) : Middleware(MakeNodeInterface(std::forward<Node>(node))) {}

  explicit Middleware(std::shared_ptr<NodeInterface> node_interface);

  /// \copydoc beluga_estimation::MiddlewareInterface::GetBackendName
  std::string_view GetBackendName() const override;

  /// \copydoc beluga_estimation::MiddlewareInterface::GetBackendObject
  void* GetBackendObject() override;

  /// \copydoc beluga_estimation::MiddlewareInterface::GetParametersInterface
  beluga_estimation::ParametersInterface* GetParametersInterface() override;

  /// \copydoc beluga_estimation::MiddlewareInterface::GetTopicsInterface
  beluga_estimation::TopicsInterface* GetTopicsInterface() override;

  /// \copydoc beluga_estimation::MiddlewareInterface::GetClockInterface
  beluga_estimation::ClockInterface* GetClockInterface() override;

  /// \copydoc beluga_estimation::MiddlewareInterface::GetTransformInterface
  beluga_estimation::TransformInterface* GetTransformInterface() override;

  /// \copydoc beluga_estimation::MiddlewareInterface::CreateRate
  std::unique_ptr<beluga_estimation::RateInterface> CreateRate(double frequency) override;
  /// \copydoc beluga_estimation::MiddlewareInterface::GetLoggerInterface
  beluga_estimation::LoggerInterface* GetLoggerInterface() override;

 private:
  std::shared_ptr<NodeInterface> node_;                             ///< The ROS 2 node base interface.
  std::unique_ptr<beluga_estimation_ros::Parameters> parameters_;   ///< The ROS 2 parameters interface.
  std::unique_ptr<beluga_estimation_ros::Topics> topics_;           ///< The ROS 2 topics interface.
  std::unique_ptr<beluga_estimation_ros::Clock> clock_;             ///< The ROS 2 clock interface.
  std::unique_ptr<beluga_estimation_ros::TF2Interface> transform_;  ///< The ROS 2 transform interface.
  std::unique_ptr<beluga_estimation_ros::Logger> logger_;           ///< The ROS 2 logger interface.
};

}  // namespace beluga_estimation_ros

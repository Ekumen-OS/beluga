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

#include <chrono>
#include <memory>
#include <string_view>

#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <beluga_estimation/middleware/transform_interface.hpp>

namespace beluga_estimation_ros {

using NodeInterface = rclcpp::node_interfaces::NodeInterfaces<ALL_RCLCPP_NODE_INTERFACES>;

/**
 * @brief ROS 2 implementation of the Beluga transform interface.
 *
 * Wraps tf2_ros functionality to query and publish coordinate transforms.
 */
class TF2Interface : public beluga_estimation::TransformInterface {
 public:
  explicit TF2Interface(std::shared_ptr<NodeInterface> node_interface);

  std::optional<beluga_estimation::SE3d> LookupTransform(
      std::string_view target_frame,
      std::string_view source_frame,
      beluga_estimation::TimePoint time,
      std::optional<std::chrono::nanoseconds> timeout = std::nullopt) const override;

  void BroadcastTransform(
      std::string_view target_frame,
      std::string_view source_frame,
      beluga_estimation::TimePoint time,
      const beluga_estimation::SE3d& transform) override;

 private:
  std::shared_ptr<NodeInterface> node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace beluga_estimation_ros

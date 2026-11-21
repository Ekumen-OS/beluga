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

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <beluga_estimation_ros/converters/types.hpp>
#include <beluga_estimation_ros/interfaces/transform.hpp>

namespace beluga_estimation_ros {

TF2Interface::TF2Interface(std::shared_ptr<NodeInterface> node_interface) : node_(std::move(node_interface)) {
  constexpr bool kSpinThread = true;  // Node may have been loaded onto a single threaded executor.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_node_clock_interface()->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_, kSpinThread);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

std::optional<beluga_estimation::SE3d> TF2Interface::LookupTransform(
    std::string_view target_frame,
    std::string_view source_frame,
    beluga_estimation::TimePoint time,
    std::optional<std::chrono::nanoseconds> timeout) const {
  try {
    const auto transform_stamped = tf_buffer_->lookupTransform(
        std::string(target_frame), std::string(source_frame),
        tf2::TimePoint{std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch())},
        tf2::Duration{timeout.value_or(std::chrono::nanoseconds::zero())});

    const auto& t = transform_stamped.transform.translation;
    const auto& q = transform_stamped.transform.rotation;

    return beluga_estimation::SE3d(Eigen::Quaterniond(q.w, q.x, q.y, q.z), Eigen::Vector3d(t.x, t.y, t.z));
  } catch (const tf2::TransformException&) {
    return std::nullopt;
  }
}

void TF2Interface::BroadcastTransform(
    std::string_view target_frame,
    std::string_view source_frame,
    beluga_estimation::TimePoint time,
    const beluga_estimation::SE3d& transform) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = beluga_estimation_ros::ToRos(time);
  msg.header.frame_id = std::string(target_frame);
  msg.child_frame_id = std::string(source_frame);

  msg.transform.translation.x = transform.translation().x();
  msg.transform.translation.y = transform.translation().y();
  msg.transform.translation.z = transform.translation().z();

  const Eigen::Quaterniond q = transform.unit_quaternion();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(msg);
}

}  // namespace beluga_estimation_ros

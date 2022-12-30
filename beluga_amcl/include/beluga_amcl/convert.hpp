// Copyright 2022 Ekumen, Inc.
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

#ifndef BELUGA_AMCL__CONVERT_HPP_
#define BELUGA_AMCL__CONVERT_HPP_

#include <tf2/utils.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sophus/se2.hpp>

namespace Sophus
{

template<class Scalar>
geometry_msgs::msg::Pose toMsg(const Sophus::SE2<Scalar> & pose)
{
  auto message = geometry_msgs::msg::Pose{};
  const double theta = pose.so2().log();
  message.position.x = pose.translation().x();
  message.position.y = pose.translation().y();
  message.position.z = 0;
  message.orientation.w = std::cos(theta / 2.);
  message.orientation.x = 0;
  message.orientation.y = 0;
  message.orientation.z = std::sin(theta / 2.);
  return message;
}

template<class Scalar>
void fromMsg(const geometry_msgs::msg::Transform & message, Sophus::SE2<Scalar> & transform)
{
  transform.translation() = Eigen::Vector2d{message.translation.x, message.translation.y};
  transform.so2() = Sophus::SO2<Scalar>{tf2::getYaw(message.rotation)};
}

}  // namespace Sophus

#endif  // BELUGA_AMCL__CONVERT_HPP_

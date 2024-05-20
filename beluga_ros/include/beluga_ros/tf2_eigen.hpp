// Copyright 2024 Ekumen, Inc.
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

#ifndef BELUGA_ROS_TF2_EIGEN_HPP
#define BELUGA_ROS_TF2_EIGEN_HPP

#if BELUGA_ROS_VERSION == 2
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif BELUGA_ROS_VERSION == 1
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#error BELUGA_ROS_VERSION is not defined or invalid
#endif

#include <beluga_ros/messages.hpp>

#include <Eigen/Core>

/**
 * \file
 * \brief Message conversion API overloads for `Eigen` types.
 */

/// `tf2` namespace extension for message conversion overload resolution.
namespace tf2 {

/// Converts an Eigen Vector2 type to a Point message.
/**
 * \param in The Eigen Vector2 element to convert.
 * \return The converted Point message.
 */
template <class Scalar>
inline beluga_ros::msg::Point toMsg(const Eigen::Vector2<Scalar>& in) {
  beluga_ros::msg::Point out;
  out.x = static_cast<double>(in.x());
  out.y = static_cast<double>(in.y());
  return out;
}

}  // namespace tf2

#endif  // BELUGA_ROS_TF2_EIGEN_HPP

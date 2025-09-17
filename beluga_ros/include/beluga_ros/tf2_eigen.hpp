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

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point.hpp>

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
inline geometry_msgs::msg::Point toMsg(const Eigen::Matrix<Scalar, 2, 1>& in) {
  geometry_msgs::msg::Point out;
  out.x = static_cast<double>(in.x());
  out.y = static_cast<double>(in.y());
  return out;
}

}  // namespace tf2

#endif  // BELUGA_ROS_TF2_EIGEN_HPP

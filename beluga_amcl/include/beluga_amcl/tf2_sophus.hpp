// Copyright 2022-2023 Ekumen, Inc.
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

#ifndef BELUGA_AMCL_TF2_SOPHUS_HPP
#define BELUGA_AMCL_TF2_SOPHUS_HPP

#include <tf2/convert.h>
#include <tf2/utils.h>

#if BELUGA_AMCL_ROS_VERSION == 2
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif BELUGA_AMCL_ROS_VERSION == 1
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#error BELUGA_AMCL_ROS_VERSION is not defined or invalid
#endif

#include <beluga_amcl/ros_interfaces.hpp>

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/types.hpp>

namespace tf2 {

/// Converts a Sophus SE2 type to a Pose message.
/**
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 *
 * \param in The Sophus SE2 element to convert.
 * \param out The pose converted to a Pose message.
 * \return The pose converted to a Pose message.
 */
template <class Scalar>
inline beluga_amcl::messages::Pose& toMsg(const Sophus::SE2<Scalar>& in, beluga_amcl::messages::Pose& out) {
  const double theta = in.so2().log();
  out.position.x = in.translation().x();
  out.position.y = in.translation().y();
  out.position.z = 0;
  out.orientation.w = std::cos(theta / 2.);
  out.orientation.x = 0;
  out.orientation.y = 0;
  out.orientation.z = std::sin(theta / 2.);
  return out;
}

/// Converts a Sophus SE3 type to a Pose message.
/**
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 *
 * \param in The Sophus SE3 element to convert.
 * \param out The pose converted to a Pose message.
 * \return The pose converted to a Pose message.
 */
template <class Scalar>
inline beluga_amcl::messages::Pose& toMsg(const Sophus::SE3<Scalar>& in, beluga_amcl::messages::Pose& out) {
  out.position.x = in.translation().x();
  out.position.y = in.translation().y();
  out.position.z = in.translation().z();
  out.orientation.w = in.so3().unit_quaternion().w();
  out.orientation.x = in.so3().unit_quaternion().x();
  out.orientation.y = in.so3().unit_quaternion().y();
  out.orientation.z = in.so3().unit_quaternion().z();
  return out;
}

/// Converts a Sophus SE2 type to a Transform message.
/**
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 *
 * \param in The Sophus SE2 element to convert.
 * \return The transform converted to a Transform message.
 */
template <class Scalar>
inline beluga_amcl::messages::Transform toMsg(const Sophus::SE2<Scalar>& in) {
  auto msg = beluga_amcl::messages::Transform{};
  const double theta = in.so2().log();
  msg.translation.x = in.translation().x();
  msg.translation.y = in.translation().y();
  msg.translation.z = 0;
  msg.rotation.w = std::cos(theta / 2.);
  msg.rotation.x = 0;
  msg.rotation.y = 0;
  msg.rotation.z = std::sin(theta / 2.);
  return msg;
}

/// Converts a Sophus SE3 type to a Transform message.
/**
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 *
 * \param in The Sophus SE3 element to convert.
 * \return The transform converted to a Transform message.
 */
template <class Scalar>
inline beluga_amcl::messages::Transform toMsg(const Sophus::SE3<Scalar>& in) {
  auto msg = beluga_amcl::messages::Transform{};
  msg.translation.x = in.translation().x();
  msg.translation.y = in.translation().y();
  msg.translation.z = in.translation().z();
  msg.rotation.w = in.so3().unit_quaternion().w();
  msg.rotation.x = in.so3().unit_quaternion().x();
  msg.rotation.y = in.so3().unit_quaternion().y();
  msg.rotation.z = in.so3().unit_quaternion().z();
  return msg;
}

/// Converts a Transform message type to a Sophus SE2 type.
/**
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 *
 * \param msg The transform message to convert.
 * \param out The transform converted to a Sophus SE2 element.
 */
template <class Scalar>
inline void fromMsg(const beluga_amcl::messages::Transform& msg, Sophus::SE2<Scalar>& out) {
  out.translation() = Sophus::Vector2<Scalar>{
      msg.translation.x,
      msg.translation.y,
  };
  out.so2() = Sophus::SO2<Scalar>{tf2::getYaw(msg.rotation)};
}

/// Converts a Transform message type to a Sophus SE3 type.
/**
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 *
 * \param msg The transform message to convert.
 * \param out The transform converted to a Sophus SE3 element.
 */
template <class Scalar>
inline void fromMsg(const beluga_amcl::messages::Transform& msg, Sophus::SE3<Scalar>& out) {
  out.translation() = Sophus::Vector3<Scalar>{
      msg.translation.x,
      msg.translation.y,
      msg.translation.z,
  };
  out.so3() = Sophus::SO3<Scalar>{Eigen::Quaternion<Scalar>{
      msg.rotation.w,
      msg.rotation.x,
      msg.rotation.y,
      msg.rotation.z,
  }};
}

/// Converts a Pose message type to a Sophus SE2 type.
/**
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 *
 * \param msg The pose message to convert.
 * \param out The pose converted to a Sophus SE2 element.
 */
template <class Scalar>
inline void fromMsg(const beluga_amcl::messages::Pose& msg, Sophus::SE2<Scalar>& out) {
  out.translation() = Sophus::Vector2<Scalar>{
      msg.position.x,
      msg.position.y,
  };
  out.so2() = Sophus::SO2<Scalar>{tf2::getYaw(msg.orientation)};
}

/// Converts a Pose message type to a Sophus SE3 type.
/**
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 *
 * \param msg The pose message to convert.
 * \param out The pose converted to a Sophus SE3 element.
 */
template <class Scalar>
inline void fromMsg(const beluga_amcl::messages::Pose& msg, Sophus::SE3<Scalar>& out) {
  out.translation() = Sophus::Vector3<Scalar>{
      msg.position.x,
      msg.position.y,
      msg.position.z,
  };
  out.so3() = Sophus::SO3<Scalar>{Eigen::Quaternion<Scalar>{
      msg.orientation.w,
      msg.orientation.x,
      msg.orientation.y,
      msg.orientation.z,
  }};
}

/// Converts a Sophus (ie. Eigen) 3x3 covariance matrix to a 6x6 row-major array.
/**
 * \param in A Sophus (ie. Eigen) 3x3 covariance matrix of a 2D pose (x, y, yaw).
 * \parma out A row-major array of 36 covariance values of a 3D pose.
 * \return a reference to `out`.
 */
template <template <typename, std::size_t> class Array, typename Scalar>
inline Array<Scalar, 36>& covarianceEigenToRowMajor(const Sophus::Matrix3<Scalar>& in, Array<Scalar, 36>& out) {
  out.fill(0);
  out[0] = in.coeff(0, 0);
  out[1] = in.coeff(0, 1);
  out[5] = in.coeff(0, 2);
  out[6] = in.coeff(1, 0);
  out[7] = in.coeff(1, 1);
  out[11] = in.coeff(1, 2);
  out[30] = in.coeff(2, 0);
  out[31] = in.coeff(2, 1);
  out[35] = in.coeff(2, 2);
  return out;
}

/// Converts a 6x6 row-major array to an Eigen 3x3 covariance matrix.
/**
 * \param in A row-major array of 36 covariance values of a 3D pose.
 * \param out A Sophus (ie. Eigen) 3x3 covariance matrix of a 2D pose (x, y, yaw).
 * \return A Sophus (ie. Eigen) 3x3 covariance matrix of a 2D pose (x, y, yaw).
 */
template <template <typename, std::size_t> class Array, typename Scalar>
inline Sophus::Matrix3<Scalar>& covarianceRowMajorToEigen(const Array<Scalar, 36>& in, Sophus::Matrix3<Scalar>& out) {
  out.coeffRef(0, 0) = in[0];
  out.coeffRef(0, 1) = in[1];
  out.coeffRef(0, 2) = in[5];
  out.coeffRef(1, 0) = in[6];
  out.coeffRef(1, 1) = in[7];
  out.coeffRef(1, 2) = in[11];
  out.coeffRef(2, 0) = in[30];
  out.coeffRef(2, 1) = in[31];
  out.coeffRef(2, 2) = in[35];
  return out;
}

}  // namespace tf2

namespace Sophus {  // NOLINT(readability-identifier-naming)
// The following conversion functions need to be inside the Sophus namespace to make
// them findable by ADL when calling tf2::convert().

template <class Scalar>
inline beluga_amcl::messages::Transform toMsg(const Sophus::SE2<Scalar>& in) {
  return tf2::toMsg(in);
}

template <class Scalar>
inline beluga_amcl::messages::Transform toMsg(const Sophus::SE3<Scalar>& in) {
  return tf2::toMsg(in);
}

template <class Scalar>
inline void fromMsg(const beluga_amcl::messages::Transform& msg, Sophus::SE2<Scalar>& out) {
  tf2::fromMsg(msg, out);
}

template <class Scalar>
inline void fromMsg(const beluga_amcl::messages::Transform& msg, Sophus::SE3<Scalar>& out) {
  tf2::fromMsg(msg, out);
}

template <class Scalar>
inline void fromMsg(const beluga_amcl::messages::Pose& msg, Sophus::SE2<Scalar>& out) {
  tf2::fromMsg(msg, out);
}

template <class Scalar>
inline void fromMsg(const beluga_amcl::messages::Pose& msg, Sophus::SE3<Scalar>& out) {
  tf2::fromMsg(msg, out);
}

}  // namespace Sophus

#endif  // BELUGA_AMCL_TF2_SOPHUS_HPP

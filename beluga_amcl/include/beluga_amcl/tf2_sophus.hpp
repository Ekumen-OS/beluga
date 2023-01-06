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

#ifndef BELUGA_AMCL__TF2_SOPHUS_HPP_
#define BELUGA_AMCL__TF2_SOPHUS_HPP_

#include <tf2/convert.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

namespace tf2
{

/**
 * @brief Convert a Sophus SE2 type to a Pose message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * @param in The Sophus SE2 to convert.
 * @param out The pose converted to a Pose message.
 * @return The pose converted to a Pose message.
 */
template<class Scalar>
inline geometry_msgs::msg::Pose & toMsg(
  const Sophus::SE2<Scalar> & in,
  geometry_msgs::msg::Pose & out)
{
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

/**
 * @brief Convert a Sophus SE3 type to a Pose message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * @param in The Sophus SE3 to convert.
 * @param out The pose converted to a Pose message.
 * @return The pose converted to a Pose message.
 */
template<class Scalar>
inline geometry_msgs::msg::Pose & toMsg(
  const Sophus::SE3<Scalar> & in,
  geometry_msgs::msg::Pose & out)
{
  out.position.x = in.translation().x();
  out.position.y = in.translation().y();
  out.position.z = in.translation().z();
  out.orientation.w = in.so3().unit_quaternion().w();
  out.orientation.x = in.so3().unit_quaternion().x();
  out.orientation.y = in.so3().unit_quaternion().y();
  out.orientation.z = in.so3().unit_quaternion().z();
  return out;
}

/**
 * @brief Convert a Sophus SE2 type to a Transform message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * @param in The Sophus SE2 to convert.
 * @return The transform converted to a Transform message.
 */
template<class Scalar>
inline geometry_msgs::msg::Transform toMsg(const Sophus::SE2<Scalar> & in)
{
  auto msg = geometry_msgs::msg::Transform{};
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

/**
 * @brief Convert a Sophus SE3 type to a Transform message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * @param in The Sophus SE3 to convert.
 * @return The transform converted to a Transform message.
 */
template<class Scalar>
inline geometry_msgs::msg::Transform toMsg(const Sophus::SE3<Scalar> & in)
{
  auto msg = geometry_msgs::msg::Transform{};
  msg.translation.x = in.translation().x();
  msg.translation.y = in.translation().y();
  msg.translation.z = in.translation().z();
  msg.rotation.w = in.so3().unit_quaternion().w();
  msg.rotation.x = in.so3().unit_quaternion().x();
  msg.rotation.y = in.so3().unit_quaternion().y();
  msg.rotation.z = in.so3().unit_quaternion().z();
  return msg;
}

/**
 * @brief Convert a Transform message type to a Sophus-specific SE2 type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * @param msg The Transform message to convert.
 * @param out The transform converted to a Sophus SE2.
 */
template<class Scalar>
inline void fromMsg(const geometry_msgs::msg::Transform & msg, Sophus::SE2<Scalar> & out)
{
  out.translation() = Eigen::Vector2<Scalar>{
    msg.translation.x,
    msg.translation.y,
  };
  out.so2() = Sophus::SO2<Scalar>{tf2::getYaw(msg.rotation)};
}

/**
 * @brief Convert a Transform message type to a Sophus-specific SE3 type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * @param msg The Transform message to convert.
 * @param out The transform converted to a Sophus SE3.
 */
template<class Scalar>
inline void fromMsg(const geometry_msgs::msg::Transform & msg, Sophus::SE3<Scalar> & out)
{
  out.translation() = Eigen::Vector3<Scalar>{
    msg.translation.x,
    msg.translation.y,
    msg.translation.z,
  };
  out.so3() = Sophus::SO3<Scalar>{
    Eigen::Quaternion<Scalar>{
      msg.rotation.w,
      msg.rotation.x,
      msg.rotation.y,
      msg.rotation.z,
    }
  };
}

/**
 * @brief Convert a Pose message type to a Sophus-specific SE2 type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * @param msg The Pose message to convert.
 * @param out The pose converted to a Sophus SE2.
 */
template<class Scalar>
inline void fromMsg(const geometry_msgs::msg::Pose & msg, Sophus::SE2<Scalar> & out)
{
  out.translation() = Eigen::Vector2<Scalar>{
    msg.position.x,
    msg.position.y,
  };
  out.so2() = Sophus::SO2<Scalar>{tf2::getYaw(msg.orientation)};
}

/**
 * @brief Convert a Pose message type to a Sophus-specific SE3 type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * @param msg The Pose message to convert.
 * @param out The pose converted to a Sophus SE3.
 */
template<class Scalar>
inline void fromMsg(const geometry_msgs::msg::Pose & msg, Sophus::SE3<Scalar> & out)
{
  out.translation() = Eigen::Vector3<Scalar>{
    msg.position.x,
    msg.position.y,
    msg.position.z,
  };
  out.so3() = Sophus::SO3<Scalar>{
    Eigen::Quaternion<Scalar>{
      msg.orientation.w,
      msg.orientation.x,
      msg.orientation.y,
      msg.orientation.z,
    }
  };
}

/**
 * @brief Function that converts from an Eigen 3x3 matrix representation of a 2D
 * covariance matrix to a 6x6 row-major representation in 3D.
 * @param in An Eigen 3x3 matrix representation of a 2D covariance.
 * @return A row-major array of 36 covariance values.
 */
template<class Scalar>
inline std::array<Scalar, 36> covarianceEigenToRowMajor(const Eigen::Matrix3<Scalar> & in)
{
  auto covariance = std::array<Scalar, 36>{};
  covariance.fill(0);
  covariance[0] = in.coeff(0, 0);
  covariance[1] = in.coeff(0, 1);
  covariance[5] = in.coeff(0, 2);
  covariance[6] = in.coeff(1, 0);
  covariance[7] = in.coeff(1, 1);
  covariance[11] = in.coeff(1, 2);
  covariance[30] = in.coeff(2, 0);
  covariance[31] = in.coeff(2, 1);
  covariance[35] = in.coeff(2, 2);
  return covariance;
}

/**
 * @brief Function that converts from row-major array representing a 3D
 * covariance matrix to an Eigen 3x3 matrix.
 * @param in A row-major array of 36 covariance values.
 * @param out An Eigen 3x3 matrix representation of a 2D covariance.
 * @return An Eigen 3x3 matrix representation of a 2D covariance.
 */
template<class Scalar>
inline Eigen::Matrix3<Scalar> & covarianceRowMajorToEigen(
  const std::array<Scalar, 36> & in,
  Eigen::Matrix3<Scalar> & out)
{
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

namespace Sophus
{
// The following conversion functions need to be inside the Sophus namespace to make
// them findable by ADL when calling tf2::convert().

template<class Scalar>
inline geometry_msgs::msg::Transform toMsg(const Sophus::SE2<Scalar> & in)
{
  return tf2::toMsg(in);
}

template<class Scalar>
inline geometry_msgs::msg::Transform toMsg(const Sophus::SE3<Scalar> & in)
{
  return tf2::toMsg(in);
}

template<class Scalar>
inline void fromMsg(const geometry_msgs::msg::Transform & msg, Sophus::SE2<Scalar> & out)
{
  tf2::fromMsg(msg, out);
}

template<class Scalar>
inline void fromMsg(const geometry_msgs::msg::Transform & msg, Sophus::SE3<Scalar> & out)
{
  tf2::fromMsg(msg, out);
}

template<class Scalar>
inline void fromMsg(const geometry_msgs::msg::Pose & msg, Sophus::SE2<Scalar> & out)
{
  tf2::fromMsg(msg, out);
}

template<class Scalar>
inline void fromMsg(const geometry_msgs::msg::Pose & msg, Sophus::SE3<Scalar> & out)
{
  tf2::fromMsg(msg, out);
}

}  // namespace Sophus

#endif  // BELUGA_AMCL__TF2_SOPHUS_HPP_

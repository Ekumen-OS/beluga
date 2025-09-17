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

#ifndef BELUGA_ROS_TF2_SOPHUS_HPP
#define BELUGA_ROS_TF2_SOPHUS_HPP

#include <beluga/algorithm/unscented_transform.hpp>
#include <beluga/eigen_compatibility.hpp>
#include <cmath>
#include <sophus/common.hpp>
#include <tf2/convert.hpp>
#include <tf2/utils.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/types.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform.hpp>

/**
 * \file
 * \brief Message conversion API overloads for `Sophus` types.
 */

namespace detail {
/// Converts a vector representing a tangent in se3 space, and returns a vector representing the same SE3 transform with
/// extrinsic RPY parametrization.
template <typename Scalar>
inline Eigen::Vector<Scalar, 6> tangential_space_to_xyz_rpy(const Eigen::Vector<Scalar, 6>& tangent) {
  // ROS covariances use extrinsic RPY parametrization, we need to convert to it from our tangent space
  // representation. We use an unscented transform to apply such transform as it's non linear.
  // See https://www.ros.org/reps/rep-0103.html#covariance-representation .
  const auto se3 = Sophus::SE3<Scalar>::exp(tangent);
  // Eigen's eulerAngles uses intrinsic rotations, but XYZ extrinsic rotation is the same as ZYX intrinsic rotation.
  // See https://pages.github.berkeley.edu/EECS-106/fa21-site/assets/discussions/D1_Rotations_soln.pdf
  // This gives (extrinsic) yaw, pitch, roll in that order.
  const Eigen::Vector<Scalar, 3> euler_angles = se3.so3().matrix().eulerAngles(2, 1, 0);
  Eigen::Vector<Scalar, 6> ret;
  ret.template head<3>() = se3.translation();
  ret[3] = euler_angles.z();
  ret[4] = euler_angles.y();
  ret[5] = euler_angles.x();
  return ret;
}
}  // namespace detail

/// `tf2` namespace extension for message conversion overload resolution.
namespace tf2 {

/// Converts a Sophus SE2 type to a Pose message.
/**
 * The canonical message for an SE2 instance is Transform. If a Pose message is needed, this
 * function must be called directly using `tf2::toMsg`, not through the `tf2::convert`
 * customization function.
 *
 * \param in The Sophus SE2 element to convert.
 * \param out The pose converted to a Pose message.
 * \return The pose converted to a Pose message.
 */
template <class Scalar>
inline geometry_msgs::msg::Pose& toMsg(const Sophus::SE2<Scalar>& in, geometry_msgs::msg::Pose& out) {
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
 * The canonical message for an SE3 instance is Transform. If a Pose message is needed, this
 * function must be called directly using `tf2::toMsg`, not through the `tf2::convert`
 * customization function.
 *
 * \param in The Sophus SE3 element to convert.
 * \param out The pose converted to a Pose message.
 * \return The pose converted to a Pose message.
 */
template <class Scalar>
inline geometry_msgs::msg::Pose& toMsg(const Sophus::SE3<Scalar>& in, geometry_msgs::msg::Pose& out) {
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
inline geometry_msgs::msg::Transform toMsg(const Sophus::SE2<Scalar>& in) {
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

/// Converts a Sophus SE3 type to a Transform message.
/**
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 *
 * \param in The Sophus SE3 element to convert.
 * \return The transform converted to a Transform message.
 */
template <class Scalar>
inline geometry_msgs::msg::Transform toMsg(const Sophus::SE3<Scalar>& in) {
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

/// Converts a Transform message type to a Sophus SE2 type.
/**
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 *
 * \param msg The transform message to convert.
 * \param out The transform converted to a Sophus SE2 element.
 */
template <class Scalar>
inline void fromMsg(const geometry_msgs::msg::Transform& msg, Sophus::SE2<Scalar>& out) {
  out.translation() = Sophus::Vector2<Scalar>{
      msg.translation.x,
      msg.translation.y,
  };
  out.so2() = Sophus::SO2<Scalar>{static_cast<Scalar>(tf2::getYaw(msg.rotation))};
}

/// Converts a Transform message type to a Sophus SE3 type.
/**
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 *
 * \param msg The transform message to convert.
 * \param out The transform converted to a Sophus SE3 element.
 */
template <class Scalar>
inline void fromMsg(const geometry_msgs::msg::Transform& msg, Sophus::SE3<Scalar>& out) {
  out.translation() = Sophus::Vector3<Scalar>{
      static_cast<Scalar>(msg.translation.x),
      static_cast<Scalar>(msg.translation.y),
      static_cast<Scalar>(msg.translation.z),
  };
  out.so3() = Sophus::SO3<Scalar>{Eigen::Quaternion<Scalar>{
      static_cast<Scalar>(msg.rotation.w),
      static_cast<Scalar>(msg.rotation.x),
      static_cast<Scalar>(msg.rotation.y),
      static_cast<Scalar>(msg.rotation.z),
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
inline void fromMsg(const geometry_msgs::msg::Pose& msg, Sophus::SE2<Scalar>& out) {
  out.translation() = Sophus::Vector2<Scalar>{
      static_cast<Scalar>(msg.position.x),
      static_cast<Scalar>(msg.position.y),
  };
  out.so2() = Sophus::SO2<Scalar>{static_cast<Scalar>(tf2::getYaw(msg.orientation))};
}

/// Converts a Pose message type to a Sophus SE3 type.
/**
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 *
 * \param msg The pose message to convert.
 * \param out The pose converted to a Sophus SE3 element.
 */
template <class Scalar>
inline void fromMsg(const geometry_msgs::msg::Pose& msg, Sophus::SE3<Scalar>& out) {
  out.translation() = Sophus::Vector3<Scalar>{
      static_cast<Scalar>(msg.position.x),
      static_cast<Scalar>(msg.position.y),
      static_cast<Scalar>(msg.position.z),
  };
  out.so3() = Sophus::SO3<Scalar>{Eigen::Quaternion<Scalar>{
      static_cast<Scalar>(msg.orientation.w),
      static_cast<Scalar>(msg.orientation.x),
      static_cast<Scalar>(msg.orientation.y),
      static_cast<Scalar>(msg.orientation.z),
  }};
}

/// Converts a Sophus (ie. Eigen) 3x3 covariance matrix to a 6x6 row-major array.
/**
 * \param in A Sophus (ie. Eigen) 3x3 covariance matrix of a 2D pose (x, y, yaw).
 * \param out A row-major array of 36 covariance values of a 3D pose.
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

/// Converts an SE3 `pose` and its covariance to a ROS message.
/// NOTE: Input covariance is expected to use tangent space parametrization, consistent with the one in beluga's \c
/// estimation libraries.
template <typename Scalar>
inline geometry_msgs::msg::PoseWithCovariance toMsg(
    const Sophus::SE3<Scalar>& in,
    const Eigen::Matrix<Scalar, 6, 6>& covariance) {
  geometry_msgs::msg::PoseWithCovariance out;
  tf2::toMsg(in, out.pose);

  // ROS covariances use extrinsic RPY parametrization, we need to convert to it from our tangent space
  // representation. We use an unscented transform to apply such transform as it's non linear.
  // See https://www.ros.org/reps/rep-0103.html#covariance-representation .

  const auto& [base_pose_in_map_xyz_rpy, base_pose_covariance_xyz_rpy] = beluga::unscented_transform(
      in.log(), covariance, detail::tangential_space_to_xyz_rpy<Scalar>, std::nullopt,
      [](const std::vector<Eigen::Vector<Scalar, 6>>& samples, const std::vector<Scalar>& weights) {
        Eigen::Vector<Scalar, 6> out = Eigen::Vector<Scalar, 6>::Zero();
        Eigen::Vector<Scalar, 2> roll_aux = Eigen::Vector<Scalar, 2>::Zero();
        Eigen::Vector<Scalar, 2> pitch_aux = Eigen::Vector<Scalar, 2>::Zero();
        Eigen::Vector<Scalar, 2> yaw_aux = Eigen::Vector<Scalar, 2>::Zero();

        for (const auto& [s, w] : ranges::views::zip(samples, weights)) {
          out.template head<3>() += s.template head<3>() * w;
          roll_aux.x() = std::sin(s[3]) * w;
          roll_aux.y() = std::cos(s[3]) * w;
          pitch_aux.x() = std::sin(s[4]) * w;
          pitch_aux.y() = std::cos(s[4]) * w;
          yaw_aux.x() = std::sin(s[5]) * w;
          yaw_aux.y() = std::cos(s[5]) * w;
        }

        out[3] = std::atan2(roll_aux.x(), roll_aux.y());
        out[4] = std::atan2(pitch_aux.x(), pitch_aux.y());
        out[5] = std::atan2(yaw_aux.x(), yaw_aux.y());
        return out;
      },
      [](const Eigen::Vector<Scalar, 6>& sample, const Eigen::Vector<Scalar, 6>& mean) {
        Eigen::Vector<Scalar, 6> out;
        const Sophus::SO3<Scalar> sample_so3(
            Sophus::SO3<Scalar>::rotZ(sample[5]) * Sophus::SO3<Scalar>::rotY(sample[4]) *
            Sophus::SO3<Scalar>::rotX(sample[3]));

        const Sophus::SO3<Scalar> mean_so3(
            Sophus::SO3<Scalar>::rotZ(mean[5]) * Sophus::SO3<Scalar>::rotY(mean[4]) *
            Sophus::SO3<Scalar>::rotX(mean[3]));

        const Sophus::SO3<Scalar> delta = mean_so3.inverse() * sample_so3;
        const Eigen::AngleAxis<Scalar> angle_axis{delta.unit_quaternion()};
        out.template head<3>() = sample.template head<3>() - mean.template head<3>();
        out.template tail<3>() = angle_axis.axis() * angle_axis.angle();
        return out;
      });

  // ROS covariance elements type is always double, and they're in RowMajor order.
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(out.covariance.data()) =
      base_pose_covariance_xyz_rpy.template cast<double>();
  return out;
}

}  // namespace tf2

/// `Sophus` namespace extension for message conversion function overload resolution (ADL enabled).
namespace Sophus {  // NOLINT(readability-identifier-naming)

/// Converts an SE2 `pose` to a `geometry_msgs/Transform` message.
template <class Scalar>
inline geometry_msgs::msg::Transform toMsg(const Sophus::SE2<Scalar>& pose) {
  return tf2::toMsg(pose);
}

/// Converts an SE3 `pose` to a `geometry_msgs/Transform` message.
template <class Scalar>
inline geometry_msgs::msg::Transform toMsg(const Sophus::SE3<Scalar>& in) {
  return tf2::toMsg(in);
}

/// Extracts an SE2 `pose` from a `geometry_msgs/Transform` `message`.
template <class Scalar>
inline void fromMsg(const geometry_msgs::msg::Transform& message, Sophus::SE2<Scalar>& pose) {
  tf2::fromMsg(message, pose);
}

/// Extracts an SE3 `pose` from a `geometry_msgs/Transform` `message`.
template <class Scalar>
inline void fromMsg(const geometry_msgs::msg::Transform& message, Sophus::SE3<Scalar>& pose) {
  tf2::fromMsg(message, pose);
}

/// Extracts an SE2 `pose` from a `geometry_msgs/Pose` `message`.
template <class Scalar>
inline void fromMsg(const geometry_msgs::msg::Pose& message, Sophus::SE2<Scalar>& pose) {
  tf2::fromMsg(message, pose);
}

/// Extracts an SE3 `pose` from a `geometry_msgs/Pose` `message`.
template <class Scalar>
inline void fromMsg(const geometry_msgs::msg::Pose& message, Sophus::SE3<Scalar>& pose) {
  tf2::fromMsg(message, pose);
}

}  // namespace Sophus

#endif  // BELUGA_ROS_TF2_SOPHUS_HPP

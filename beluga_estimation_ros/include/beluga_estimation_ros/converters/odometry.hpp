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

/**
 * @file
 * @brief Adapts middleware-specific odometry messages to the Beluga Odometry interface.
 */

#include <cmath>

#include <nav_msgs/msg/odometry.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/type_casts.hpp>
#include <beluga_estimation/msg/odometry.hpp>
#include <beluga_estimation_ros/converters/fwd.hpp>
#include <beluga_estimation_ros/converters/types.hpp>

namespace beluga_estimation_ros::msg {

namespace detail {

/**
 * @brief Converts a pose covariance from the RPY parameterization to the SE(3) tangent space.
 *
 * The ROS odometry message stores the pose covariance over the parameters
 * `[x, y, z, roll, pitch, yaw]`. Beluga represents pose covariances in the
 * SE(3) tangent space over `[x, y, z, ωx, ωy, ωz]`, where the rotation part is
 * the axis-angle (exponential coordinates) perturbation. This function applies
 * the full Jacobian of the RPY-to-tangent-space mapping:
 *
 *     Σ_tangent = J * Σ_rpy * Jᵀ
 *
 * with `J = diag(I₃, J_rpy)`, where `J_rpy` is the body-frame Jacobian
 * relating RPY rates to the body-frame angular velocity:
 *
 *     J_rpy = [ 1      0          -sinθ     ]
 *             [ 0      cosφ       cosθ sinφ ]
 *             [ 0      -sinφ      cosθ cosφ ]
 *
 * The translation block is the identity because the position perturbation is
 * the same in both parameterizations. The rotation block uses the body-frame
 * Jacobian so that the resulting tangent-space covariance is consistent with
 * the right-trivialized convention used by Beluga operations (the pose error
 * is `X⁻¹ * X_true`, expressed in the body frame).
 *
 * @note The RPY parameterization is singular at pitch = ±90° (gimbal lock),
 * where `J_rpy` degenerates. The conversion is numerically ill-conditioned in
 * that regime.
 *
 * @param orientation The orientation quaternion of the pose.
 * @param rpy_covariance The 6×6 RPY covariance `[x, y, z, roll, pitch, yaw]`.
 * @return The 6×6 covariance in the SE(3) tangent space.
 */
inline beluga_estimation::Matrix6d RpyToTangentSpaceCovariance(
    const Eigen::Quaterniond& orientation,
    const beluga_estimation::Matrix6d& rpy_covariance) {
  // Extract RPY angles in the ROS convention R = Rz(yaw) * Ry(pitch) * Rx(roll).
  // The body-frame Jacobian below does not depend on yaw.
  const Eigen::Vector3d ypr = orientation.toRotationMatrix().eulerAngles(2, 1, 0);
  const double roll = ypr(2);
  const double pitch = ypr(1);

  // Body-frame RPY Jacobian: ω_body = J_rpy * [φ̇, θ̇, ψ̇]ᵀ.
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  beluga_estimation::Matrix3d J_rpy;
  J_rpy << 1.0, 0.0, -sp, 0.0, cr, cp * sr, 0.0, -sr, cp * cr;

  // Full 6×6 Jacobian: identity on translation, J_rpy on rotation.
  beluga_estimation::Matrix6d J = beluga_estimation::Matrix6d::Zero();
  J.topLeftCorner<3, 3>().setIdentity();
  J.bottomRightCorner<3, 3>() = J_rpy;

  return J * rpy_covariance * J.transpose();
}

/**
 * @brief Template implementation of the Odometry interface backed by a nav_msgs/Odometry message.
 * @tparam Base The base Odometry interface to implement (Odometry or MutableOdometry).
 */
template <typename Base>
class OdometryImpl : public Base {
 public:
  /// @brief The underlying message type.
  using RosType = nav_msgs::msg::Odometry;
  /// @brief A shared pointer to the message, const-qualified if the interface is read-only.
  using Pointer = std::shared_ptr<std::conditional_t<Base::kIsReadOnly, const RosType, RosType>>;

  /// @brief View type for SE(3) pose data.
  using SE3View = typename Base::SE3View;
  /// @brief View type for 6D vector data.
  using Vector6View = typename Base::Vector6View;
  /// @brief View type for 6x6 matrix data.
  using Matrix6View = typename Base::Matrix6View;

  /**
   * @brief Constructs an OdometryImpl from a shared pointer to a message.
   *
   * The pose, twist, and their covariances are extracted and converted to the
   * SE(3) tangent space at construction time.
   */
  explicit OdometryImpl(Pointer message)
      : message_{std::move(message)},
        pose_{beluga_estimation::SE3d(
            beluga_estimation::SO3d(Eigen::Quaterniond(
                message_->pose.pose.orientation.w,
                message_->pose.pose.orientation.x,
                message_->pose.pose.orientation.y,
                message_->pose.pose.orientation.z)),
            beluga_estimation::Vector3d(
                message_->pose.pose.position.x,
                message_->pose.pose.position.y,
                message_->pose.pose.position.z))},
        twist_{beluga_estimation::Vector6d(
            message_->twist.twist.linear.x,
            message_->twist.twist.linear.y,
            message_->twist.twist.linear.z,
            message_->twist.twist.angular.x,
            message_->twist.twist.angular.y,
            message_->twist.twist.angular.z)} {
    // The pose covariance is stored as a row-major 6x6 flat array in the RPY
    // parameterization; convert it to the SE(3) tangent space once.
    const Eigen::Map<const beluga_estimation::Matrix<double, 6, 6>> pose_cov(message_->pose.covariance.data());
    pose_covariance_ = RpyToTangentSpaceCovariance(pose_.so3().unit_quaternion(), pose_cov);

    // The twist covariance is already in the SE(3) tangent space.
    const Eigen::Map<const beluga_estimation::Matrix<double, 6, 6>> twist_cov(message_->twist.covariance.data());
    twist_covariance_ = twist_cov;
  }

  /// @brief Default constructor. Initializes the header with a new message instance.
  OdometryImpl() : OdometryImpl(std::make_shared<RosType>()) {}

  /// @brief Constructs an OdometryImpl by copying a message.
  explicit OdometryImpl(const RosType& message) : OdometryImpl(std::make_shared<RosType>(message)) {}

  /// @brief Constructs an OdometryImpl by moving a message.
  explicit OdometryImpl(RosType&& message) : OdometryImpl(std::make_shared<RosType>(std::move(message))) {}

  /// @brief Returns a pointer to the underlying message.
  const void* GetNativeHandle() const override { return message_.get(); }

  /// @brief Returns the timestamp from the ROS 2 message.
  beluga_estimation::TimePoint GetTimestamp() const override {
    return beluga_estimation_ros::FromRos(message_->header.stamp);
  }

  /// @brief Returns the frame the pose is expressed in.
  std::string_view GetFrameId() const override { return message_->header.frame_id; }

  /// @brief Returns the identifier of the body whose pose and velocity this message describes.
  std::string_view GetBodyFrameId() const override { return message_->child_frame_id; }

  /// @brief Returns the estimated pose of the body frame in the reference frame.
  SE3View GetPose() const override { return SE3View(pose_.data()); }

  /// @brief Returns the estimated body-frame spatial velocity (twist).
  Vector6View GetTwist() const override { return Vector6View(twist_.data()); }

  /// @brief Returns the 6×6 pose covariance in the SE(3) tangent space.
  Matrix6View GetPoseCovariance() const override { return Matrix6View(pose_covariance_.data()); }

  /// @brief Returns the 6×6 twist covariance in the SE(3) tangent space.
  Matrix6View GetTwistCovariance() const override { return Matrix6View(twist_covariance_.data()); }

 protected:
  /// @brief The shared message instance.
  Pointer message_;
  /// @brief Cached pose as an SE(3) element.
  beluga_estimation::SE3d pose_;
  /// @brief Cached twist as a 6D spatial velocity vector.
  beluga_estimation::Vector6d twist_;
  /// @brief Cached 6×6 pose covariance in the SE(3) tangent space.
  beluga_estimation::Matrix6d pose_covariance_;
  /// @brief Cached 6×6 twist covariance in the SE(3) tangent space.
  beluga_estimation::Matrix6d twist_covariance_;
};

}  // namespace detail

/**
 * @brief Read-only implementation of the Odometry interface for nav_msgs/Odometry.
 */
class Odometry : public detail::OdometryImpl<beluga_estimation::msg::Odometry> {
 public:
  /// @brief Type alias for the base implementation.
  using Base = detail::OdometryImpl<beluga_estimation::msg::Odometry>;
  using Base::Base;
};

/**
 * @brief Mutable implementation of the Odometry interface for nav_msgs/Odometry.
 */
class MutableOdometry : public detail::OdometryImpl<beluga_estimation::msg::MutableOdometry> {
 public:
  /// @brief Type alias for the base implementation.
  using Base = detail::OdometryImpl<beluga_estimation::msg::MutableOdometry>;
  using Base::Base;

  /**
   * @brief Conversion operator to a read-only Odometry view.
   * @return An Odometry instance sharing the same underlying message.
   */
  operator beluga_estimation_ros::msg::Odometry() const { return beluga_estimation_ros::msg::Odometry(this->message_); }
};

}  // namespace beluga_estimation_ros::msg

namespace beluga_estimation_ros {

/**
 * @brief Specialization of TypeConverter for beluga_estimation::msg::Odometry.
 */
template <>
struct TypeConverter<beluga_estimation::msg::Odometry> {
  /**
   * @brief Converts a Beluga Odometry to its underlying nav_msgs representation.
   *
   * This performs a reinterpret_cast of the native handle.
   */
  static nav_msgs::msg::Odometry ToRos(const beluga_estimation::msg::Odometry& message) {
    return *reinterpret_cast<const nav_msgs::msg::Odometry*>(message.GetNativeHandle());
  }

  static beluga_estimation::msg::Odometry FromRos(const nav_msgs::msg::Odometry&) = delete;
};

/// @brief Specialization of TypeConverter for nav_msgs::msg::Odometry.
template <>
struct TypeConverter<nav_msgs::msg::Odometry> : TypeConverter<beluga_estimation::msg::Odometry> {};

}  // namespace beluga_estimation_ros

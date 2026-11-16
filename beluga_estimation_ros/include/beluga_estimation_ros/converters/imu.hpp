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
 * @brief Provides ROS 2 implementations for the Imu message interface.
 */

#include <sensor_msgs/msg/imu.hpp>

#include <beluga_estimation/common/type_casts.hpp>
#include <beluga_estimation/msg/imu.hpp>
#include <beluga_estimation_ros/converters/fwd.hpp>
#include <beluga_estimation_ros/converters/types.hpp>

namespace beluga_estimation_ros::msg {

namespace detail {

/**
 * @brief Template implementation of the Imu interface for ROS 2.
 * @tparam B The base Imu interface to implement (Imu or MutableImu).
 */
template <typename Base>
class ImuImpl : public Base {
 public:
  /// @brief The underlying ROS 2 message type.
  using RosType = sensor_msgs::msg::Imu;
  /// @brief A shared pointer to the ROS 2 message, const-qualified if the interface is read-only.
  using Pointer = std::shared_ptr<std::conditional_t<Base::kIsReadOnly, const RosType, RosType>>;

  /// @brief View type for quaternion data.
  using QuaternionView = typename Base::QuaternionView;
  /// @brief View type for 3D vector data.
  using Vector3View = typename Base::Vector3View;
  /// @brief View type for 3x3 matrix data.
  using Matrix3View = typename Base::Matrix3View;

  /**
   * @brief Constructs an ImuImpl from a shared pointer to a ROS 2 message.
   */
  explicit ImuImpl(Pointer message)
      : message_{std::move(message)},
        orientation_{
            message_->orientation.w,
            message_->orientation.x,
            message_->orientation.y,
            message_->orientation.z,
        },
        angular_velocity_{
            message_->angular_velocity.x,
            message_->angular_velocity.y,
            message_->angular_velocity.z,
        },
        linear_acceleration_{
            message_->linear_acceleration.x,
            message_->linear_acceleration.y,
            message_->linear_acceleration.z,
        } {}

  /// @brief Default constructor. Initializes the header with a new ROS 2 message instance.
  ImuImpl() : ImuImpl(std::make_shared<RosType>()) {}

  /// @brief Constructs an ImuImpl by copying a ROS 2 message.
  explicit ImuImpl(const RosType& message) : ImuImpl(std::make_shared<RosType>(message)) {}

  /// @brief Constructs an ImuImpl by moving a ROS 2 message.
  explicit ImuImpl(RosType&& message) : ImuImpl(std::make_shared<RosType>(std::move(message))) {}

  /// @brief Returns a pointer to the underlying ROS 2 message.
  const void* GetNativeHandle() const override { return message_.get(); }

  /// @brief Returns the timestamp from the ROS 2 message.
  beluga_estimation::TimePoint GetTimestamp() const override {
    return beluga_estimation_ros::FromRos(message_->header.stamp);
  }

  /// @brief Returns the frame the measurements are expressed in.
  std::string_view GetFrameId() const override { return message_->header.frame_id; }

  /// @brief Returns the orientation estimate.
  QuaternionView GetOrientation() const override { return QuaternionView(orientation_.coeffs().data()); }

  /// @brief Returns the angular velocity.
  Vector3View GetAngularVelocity() const override { return Vector3View(angular_velocity_.data()); }

  /// @brief Returns the linear acceleration.
  Vector3View GetLinearAcceleration() const override { return Vector3View(linear_acceleration_.data()); }

  /// @brief Returns the orientation covariance matrix.
  Matrix3View GetOrientationCovariance() const override { return Matrix3View(message_->orientation_covariance.data()); }

  /// @brief Returns the angular velocity covariance matrix.
  Matrix3View GetAngularVelocityCovariance() const override {
    return Matrix3View(message_->angular_velocity_covariance.data());
  }

  /// @brief Returns the linear acceleration covariance matrix.
  Matrix3View GetLinearAccelerationCovariance() const override {
    return Matrix3View(message_->linear_acceleration_covariance.data());
  }

 protected:
  /// @brief The shared ROS 2 message instance.
  Pointer message_;
  /// @brief Cached orientation as an Eigen quaternion.
  Eigen::Quaterniond orientation_;
  /// @brief Cached angular velocity as an Eigen vector.
  Eigen::Vector3d angular_velocity_;
  /// @brief Cached linear acceleration as an Eigen vector.
  Eigen::Vector3d linear_acceleration_;
};

}  // namespace detail

/**
 * @brief Read-only implementation of the Imu interface for ROS 2.
 */
class Imu : public detail::ImuImpl<beluga_estimation::msg::Imu> {
 public:
  /// @brief Type alias for the base implementation.
  using Base = detail::ImuImpl<beluga_estimation::msg::Imu>;
  using Base::Base;
};

/**
 * @brief Mutable implementation of the Imu interface for ROS 2.
 */
class MutableImu : public detail::ImuImpl<beluga_estimation::msg::MutableImu> {
 public:
  /// @brief Type alias for the base implementation.
  using Base = detail::ImuImpl<beluga_estimation::msg::MutableImu>;
  using Base::Base;

  /**
   * @brief Conversion operator to a read-only Imu view.
   * @return An Imu instance sharing the same underlying message.
   */
  operator beluga_estimation_ros::msg::Imu() const { return beluga_estimation_ros::msg::Imu(this->message_); }
};

}  // namespace beluga_estimation_ros::msg

namespace beluga_estimation_ros {

/**
 * @brief Specialization of TypeConverter for beluga_estimation::msg::Imu.
 */
template <>
struct TypeConverter<beluga_estimation::msg::Imu> {
  /**
   * @brief Converts a Beluga Imu to its ROS 2 representation.
   *
   * This performs a reinterpret_cast of the native handle.
   */
  static sensor_msgs::msg::Imu ToRos(const beluga_estimation::msg::Imu& message) {
    return *reinterpret_cast<const sensor_msgs::msg::Imu*>(message.GetNativeHandle());
  }

  static beluga_estimation::msg::Imu FromRos(const sensor_msgs::msg::Imu&) = delete;
};

/// @brief Specialization of TypeConverter for sensor_msgs::msg::Imu.
template <>
struct TypeConverter<sensor_msgs::msg::Imu> : TypeConverter<beluga_estimation::msg::Imu> {};

}  // namespace beluga_estimation_ros

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
 * @brief Defines the Imu message interface.
 */

#include <string_view>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/message.hpp>
#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation::msg {

/**
 * @brief Interface for an Inertial Measurement Unit (IMU) message.
 *
 * This class provides read-only access to IMU data, including orientation,
 * angular velocity, and linear acceleration, along with their respective
 * covariances. Measurements are expressed in the sensor frame identified by
 * @ref GetFrameId.
 */
class Imu : public Message {
 public:
  /// @brief Type alias for the base message type.
  using BaseType = Imu;

  /// @brief View type for quaternion data.
  using QuaternionView = beluga_estimation::Map<const beluga_estimation::Quaterniond, Eigen::Unaligned>;
  /// @brief View type for 3D vector data.
  using Vector3View = beluga_estimation::Map<const beluga_estimation::Vector3d, Eigen::Unaligned>;
  /// @brief View type for 3x3 matrix data.
  using Matrix3View = beluga_estimation::Map<const beluga_estimation::Matrix3d, Eigen::Unaligned>;

  /// @brief Indicates if this class is a read-only interface.
  static constexpr bool kIsReadOnly = true;

  /**
   * @brief Returns the static unique identifier for the Imu message type.
   * @return The `MessageId` for Imu.
   */
  static constexpr MessageId GetStaticId() { return HashStringView("beluga_estimation::msg::Imu"); }

  /**
   * @brief Returns the unique identifier for the message type.
   * @return The `MessageId` associated with this message's type.
   */
  MessageId GetId() const final { return GetStaticId(); }

  /**
   * @brief Returns the timestamp associated with the message.
   * @return The `TimePoint` when the message was created or captured.
   */
  virtual beluga_estimation::TimePoint GetTimestamp() const = 0;

  /**
   * @brief Returns the identifier of the frame the measurements are expressed in.
   * @return A `std::string_view` of the sensor frame ID.
   */
  virtual std::string_view GetFrameId() const = 0;

  /**
   * @brief Returns the orientation estimate.
   * @return A `QuaternionView` of the orientation.
   */
  virtual QuaternionView GetOrientation() const = 0;

  /**
   * @brief Returns the angular velocity.
   * @return A `Vector3View` of the angular velocity.
   */
  virtual Vector3View GetAngularVelocity() const = 0;

  /**
   * @brief Returns the linear acceleration.
   * @return A `Vector3View` of the linear acceleration.
   */
  virtual Vector3View GetLinearAcceleration() const = 0;

  /**
   * @brief Returns the orientation covariance matrix.
   * @return A `Matrix3View` of the 3x3 orientation covariance.
   */
  virtual Matrix3View GetOrientationCovariance() const = 0;

  /**
   * @brief Returns the angular velocity covariance matrix.
   * @return A `Matrix3View` of the 3x3 angular velocity covariance.
   */
  virtual Matrix3View GetAngularVelocityCovariance() const = 0;

  /**
   * @brief Returns the linear acceleration covariance matrix.
   * @return A `Matrix3View` of the 3x3 linear acceleration covariance.
   */
  virtual Matrix3View GetLinearAccelerationCovariance() const = 0;
};

/**
 * @brief Interface for a mutable IMU message.
 *
 * This class extends `Imu` to provide access to mutable components of the message.
 */
class MutableImu : public Imu {
 public:
  /// @brief Indicates if this class is a view. False for mutable interfaces.
  static constexpr bool kIsReadOnly = false;

  // TODO: Complete mutable interface.
};

}  // namespace beluga_estimation::msg

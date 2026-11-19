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
 * @brief Defines the Odometry message interface.
 */

#include <string_view>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/message.hpp>
#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation::msg {

/**
 * @brief Interface for an odometry message.
 *
 * Provides read-only access to odometry data: the estimated pose and twist of
 * a body frame, and their covariances, all expressed in the SE(3) tangent
 * space. The pose is the pose of the body frame in the reference frame
 * identified by @ref GetFrameId; the twist is the body-frame spatial velocity.
 */
class Odometry : public Message {
 public:
  /// @brief Type alias for the base message type.
  using BaseType = Odometry;

  /// @brief View type for SE(3) pose data.
  using SE3View = beluga_estimation::Map<const beluga_estimation::SE3d, Eigen::Unaligned>;
  /// @brief View type for 6D vector data.
  using Vector6View = beluga_estimation::Map<const beluga_estimation::Vector6d, Eigen::Unaligned>;
  /// @brief View type for 6x6 matrix data.
  using Matrix6View = beluga_estimation::Map<const beluga_estimation::Matrix6d, Eigen::Unaligned>;

  /// @brief Indicates if this class is a read-only interface.
  static constexpr bool kIsReadOnly = true;

  /**
   * @brief Returns the static unique identifier for the Odometry message type.
   * @return The `MessageId` for Odometry.
   */
  static constexpr MessageId GetStaticId() { return HashStringView("beluga_estimation::msg::Odometry"); }

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
   * @brief Returns the identifier of the frame the pose is expressed in.
   * @return A `std::string_view` of the reference frame ID.
   */
  virtual std::string_view GetFrameId() const = 0;

  /**
   * @brief Returns the identifier of the body whose pose and twist this message describes.
   * @return A `std::string_view` of the body frame ID.
   */
  virtual std::string_view GetBodyFrameId() const = 0;

  /**
   * @brief Returns the estimated pose of the body frame in the reference frame.
   * @return A view of the pose as an SE(3) element.
   */
  virtual SE3View GetPose() const = 0;

  /**
   * @brief Returns the estimated body-frame spatial velocity (twist).
   * @return A view of the twist as a 6D vector `[vx, vy, vz, ωx, ωy, ωz]`.
   */
  virtual Vector6View GetTwist() const = 0;

  /**
   * @brief Returns the 6×6 pose covariance in the SE(3) tangent space.
   *
   * The covariance is stored in row-major order over
   * `[x, y, z, ωx, ωy, ωz]`, including cross-covariances between translation
   * and rotation.
   */
  virtual Matrix6View GetPoseCovariance() const = 0;

  /**
   * @brief Returns the 6×6 twist covariance in the SE(3) tangent space.
   *
   * The covariance is stored in row-major order over
   * `[vx, vy, vz, ωx, ωy, ωz]`, including cross-covariances between linear
   * and angular velocity.
   */
  virtual Matrix6View GetTwistCovariance() const = 0;
};

/**
 * @brief Interface for a mutable odometry message.
 *
 * This class extends `Odometry` to provide access to mutable components of the message.
 */
class MutableOdometry : public Odometry {
 public:
  /// @brief Indicates if this class is a view. False for mutable interfaces.
  static constexpr bool kIsReadOnly = false;

  // TODO: Complete mutable interface.
};

}  // namespace beluga_estimation::msg

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
 * @brief Concrete mocks for beluga_estimation::msg::Header and beluga_estimation::msg::Odometry for use in tests.
 */

#include <string>
#include <string_view>
#include <utility>

#include <Eigen/Geometry>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/msg/header.hpp>
#include <beluga_estimation/msg/odometry.hpp>

namespace beluga_estimation {

class MockHeader : public beluga_estimation::msg::Header {
 public:
  explicit MockHeader(beluga_estimation::TimePoint stamp) : stamp_(stamp) {}

  beluga_estimation::TimePoint GetStamp() const override { return stamp_; }
  std::string_view GetFrameId() const override { return "odom"; }
  const void* GetNativeHandle() const override { return nullptr; }

 private:
  beluga_estimation::TimePoint stamp_;
};

/// @brief Concrete odometry message built from an SE(2) pose and covariance matrices.
class MockOdometry : public beluga_estimation::msg::Odometry {
 public:
  /**
   * @brief Constructs a fake odometry message from 3×3 covariance blocks.
   *
   * The full 6×6 pose and twist covariances are assembled from the given
   * diagonal 3×3 blocks, with zero cross-covariance between translation and
   * rotation.
   *
   * @param t               Message timestamp.
   * @param pose            SE(2) pose to encode.
   * @param lin_vel_cov     3×3 linear velocity covariance for [vx, vy, vz].
   * @param ang_vel_cov     3×3 angular velocity covariance for [wx, wy, wz].
   * @param linear_velocity 3D linear velocity.
   * @param angular_velocity 3D angular velocity.
   * @param position_cov    3×3 position covariance for [x, y, z].
   * @param orientation_cov 3×3 orientation covariance for [ωx, ωy, ωz].
   * @param body_frame_id   Body frame identifier.
   */
  MockOdometry(
      beluga_estimation::TimePoint t,
      const beluga_estimation::SE2d& pose,
      const beluga_estimation::Matrix3d& lin_vel_cov = beluga_estimation::Matrix3d::Identity() * 0.01,
      const beluga_estimation::Matrix3d& ang_vel_cov = beluga_estimation::Matrix3d::Identity() * 0.01,
      const beluga_estimation::Vector3d& linear_velocity = beluga_estimation::Vector3d::Zero(),
      const beluga_estimation::Vector3d& angular_velocity = beluga_estimation::Vector3d::Zero(),
      const beluga_estimation::Matrix3d& position_cov = beluga_estimation::Matrix3d::Identity() * 0.01,
      const beluga_estimation::Matrix3d& orientation_cov = beluga_estimation::Matrix3d::Identity() * 0.01,
      std::string body_frame_id = "base_link")
      : timestamp_(t),
        frame_id_("odom"),
        body_frame_id_(std::move(body_frame_id)),
        pose_(MakeSE3(pose)),
        twist_(MakeTwist(linear_velocity, angular_velocity)),
        pose_cov_(MakePoseCovariance(position_cov, orientation_cov)),
        twist_cov_(MakeTwistCovariance(lin_vel_cov, ang_vel_cov)) {}

  /**
   * @brief Constructs a fake odometry message from full 6×6 covariances.
   *
   * @param t               Message timestamp.
   * @param pose            SE(2) pose to encode.
   * @param pose_cov        Full 6×6 pose covariance in the SE(3) tangent space.
   * @param twist_cov       Full 6×6 twist covariance in the SE(3) tangent space.
   * @param linear_velocity 3D linear velocity.
   * @param angular_velocity 3D angular velocity.
   * @param body_frame_id   Body frame identifier.
   * @return A mock odometry message with the given full covariances.
   */
  static MockOdometry WithFullCovariance(
      beluga_estimation::TimePoint t,
      const beluga_estimation::SE2d& pose,
      const beluga_estimation::Matrix6d& pose_cov,
      const beluga_estimation::Matrix6d& twist_cov,
      const beluga_estimation::Vector3d& linear_velocity = beluga_estimation::Vector3d::Zero(),
      const beluga_estimation::Vector3d& angular_velocity = beluga_estimation::Vector3d::Zero(),
      std::string body_frame_id = "base_link") {
    MockOdometry message(t, pose);
    message.pose_cov_ = pose_cov;
    message.twist_cov_ = twist_cov;
    message.twist_ = MakeTwist(linear_velocity, angular_velocity);
    message.body_frame_id_ = std::move(body_frame_id);
    return message;
  }

  beluga_estimation::TimePoint GetTimestamp() const override { return timestamp_; }
  std::string_view GetFrameId() const override { return frame_id_; }
  std::string_view GetBodyFrameId() const override { return body_frame_id_; }
  const void* GetNativeHandle() const override { return nullptr; }

  SE3View GetPose() const override { return SE3View(pose_.data()); }

  Vector6View GetTwist() const override { return Vector6View(twist_.data()); }

  Matrix6View GetPoseCovariance() const override { return Matrix6View(pose_cov_.data()); }

  Matrix6View GetTwistCovariance() const override { return Matrix6View(twist_cov_.data()); }

 private:
  /// @brief Builds an SE(3) pose from an SE(2) pose (planar embedding).
  static beluga_estimation::SE3d MakeSE3(const beluga_estimation::SE2d& pose) {
    return beluga_estimation::SE3d(
        beluga_estimation::SO3d::rotZ(pose.so2().log()),
        beluga_estimation::Vector3d(pose.translation().x(), pose.translation().y(), 0.0));
  }

  /// @brief Assembles a 6D twist from linear and angular velocity vectors.
  static beluga_estimation::Vector6d MakeTwist(
      const beluga_estimation::Vector3d& linear_velocity,
      const beluga_estimation::Vector3d& angular_velocity) {
    beluga_estimation::Vector6d twist;
    twist << linear_velocity, angular_velocity;
    return twist;
  }

  /// @brief Assembles a full 6×6 pose covariance from its diagonal 3×3 blocks.
  static beluga_estimation::Matrix6d MakePoseCovariance(
      const beluga_estimation::Matrix3d& position_cov,
      const beluga_estimation::Matrix3d& orientation_cov) {
    beluga_estimation::Matrix6d cov = beluga_estimation::Matrix6d::Zero();
    cov.topLeftCorner<3, 3>() = position_cov;
    cov.bottomRightCorner<3, 3>() = orientation_cov;
    return cov;
  }

  /// @brief Assembles a full 6×6 twist covariance from its diagonal 3×3 blocks.
  static beluga_estimation::Matrix6d MakeTwistCovariance(
      const beluga_estimation::Matrix3d& lin_vel_cov,
      const beluga_estimation::Matrix3d& ang_vel_cov) {
    beluga_estimation::Matrix6d cov = beluga_estimation::Matrix6d::Zero();
    cov.topLeftCorner<3, 3>() = lin_vel_cov;
    cov.bottomRightCorner<3, 3>() = ang_vel_cov;
    return cov;
  }

  beluga_estimation::TimePoint timestamp_;
  std::string frame_id_;
  std::string body_frame_id_;
  beluga_estimation::SE3d pose_;
  beluga_estimation::Vector6d twist_;
  beluga_estimation::Matrix6d pose_cov_;
  beluga_estimation::Matrix6d twist_cov_;
};

}  // namespace beluga_estimation

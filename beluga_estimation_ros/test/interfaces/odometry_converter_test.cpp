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

#include <cmath>

#include <gtest/gtest.h>

#include <nav_msgs/msg/odometry.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation_ros/converters/odometry.hpp>

namespace {

// Extracts RPY angles in the ROS convention R = Rz(yaw) * Ry(pitch) * Rx(roll).
Eigen::Vector3d ToRpy(const Eigen::Quaterniond& q) {
  return q.toRotationMatrix().eulerAngles(2, 1, 0);  // [yaw, pitch, roll]
}

// Builds the rotation matrix from RPY angles in the ROS convention.
Eigen::Matrix3d RpyToRotation(const Eigen::Vector3d& rpy) {
  return (Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// Numerically differentiates the RPY-to-axis-angle mapping at the given
// orientation, producing the body-frame Jacobian J such that
// δω_body = J * [δφ, δθ, δψ]ᵀ.
beluga_estimation::Matrix3d NumericalRpyJacobian(const Eigen::Quaterniond& q) {
  const Eigen::Vector3d rpy = ToRpy(q);
  const Eigen::Matrix3d R = q.toRotationMatrix();
  constexpr double kEps = 1e-7;
  beluga_estimation::Matrix3d J;
  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d rpy_perturbed = rpy;
    rpy_perturbed(i) += kEps;
    const Eigen::Matrix3d R_perturbed = RpyToRotation(rpy_perturbed);
    // Body-frame perturbation: R' = R * exp([δω]×), so δω = log(Rᵀ R').
    const Eigen::AngleAxisd delta(R.transpose() * R_perturbed);
    J.col(i) = (delta.angle() * delta.axis()) / kEps;
  }
  return J;
}

// Builds a nav_msgs/Odometry message with the given pose and RPY covariance.
nav_msgs::msg::Odometry MakeRosOdometry(
    const Eigen::Quaterniond& orientation,
    const beluga_estimation::Matrix6d& rpy_covariance) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp.sec = 42;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";
  msg.pose.pose.orientation.w = orientation.w();
  msg.pose.pose.orientation.x = orientation.x();
  msg.pose.pose.orientation.y = orientation.y();
  msg.pose.pose.orientation.z = orientation.z();
  msg.pose.pose.position.x = 1.0;
  msg.pose.pose.position.y = 2.0;
  msg.pose.pose.position.z = 0.5;
  msg.twist.twist.linear.x = 0.1;
  msg.twist.twist.linear.y = 0.2;
  msg.twist.twist.linear.z = 0.3;
  msg.twist.twist.angular.x = 0.4;
  msg.twist.twist.angular.y = 0.5;
  msg.twist.twist.angular.z = 0.6;
  for (int i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = rpy_covariance.data()[i];
    msg.twist.covariance[i] = rpy_covariance.data()[i];
  }
  return msg;
}

TEST(OdometryConverterTest, ExposesTimestampAndFrames) {
  const auto msg = MakeRosOdometry(Eigen::Quaterniond::Identity(), beluga_estimation::Matrix6d::Identity());
  const beluga_estimation_ros::msg::Odometry odom(msg);

  EXPECT_EQ(odom.GetTimestamp(), beluga_estimation::TimePoint(std::chrono::seconds(42)));
  EXPECT_EQ(odom.GetFrameId(), "odom");
  EXPECT_EQ(odom.GetBodyFrameId(), "base_link");
}

TEST(OdometryConverterTest, ExposesPoseAndTwist) {
  const Eigen::Quaterniond orientation(
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));
  const auto msg = MakeRosOdometry(orientation, beluga_estimation::Matrix6d::Identity());
  const beluga_estimation_ros::msg::Odometry odom(msg);

  const beluga_estimation::SE3d pose = odom.GetPose();
  EXPECT_TRUE(pose.so3().unit_quaternion().isApprox(orientation, 1e-12));
  EXPECT_TRUE(pose.translation().isApprox(beluga_estimation::Vector3d(1.0, 2.0, 0.5), 1e-12));

  const beluga_estimation::Vector6d twist = odom.GetTwist();
  EXPECT_TRUE(twist.isApprox(beluga_estimation::Vector6d(0.1, 0.2, 0.3, 0.4, 0.5, 0.6), 1e-12));
}

TEST(OdometryConverterTest, IdentityOrientationPreservesPoseCovariance) {
  // At identity orientation the RPY Jacobian is the identity, so the
  // covariance passes through unchanged.
  const beluga_estimation::Matrix6d rpy_covariance = beluga_estimation::Matrix6d::Identity();
  const auto msg = MakeRosOdometry(Eigen::Quaterniond::Identity(), rpy_covariance);
  const beluga_estimation_ros::msg::Odometry odom(msg);

  EXPECT_TRUE(odom.GetPoseCovariance().isApprox(rpy_covariance, 1e-12));
  EXPECT_TRUE(odom.GetTwistCovariance().isApprox(rpy_covariance, 1e-12));
}

TEST(OdometryConverterTest, PoseCovarianceConvertedToTangentSpace) {
  // Non-trivial orientation: the pose covariance must be transformed by the
  // full RPY-to-tangent-space Jacobian, verified against numerical
  // differentiation.
  const Eigen::Quaterniond orientation(
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));
  const beluga_estimation::Matrix6d rpy_covariance = beluga_estimation::Matrix6d::Identity();

  const auto msg = MakeRosOdometry(orientation, rpy_covariance);
  const beluga_estimation_ros::msg::Odometry odom(msg);

  // Expected: J * Σ * Jᵀ with J = diag(I₃, J_rpy).
  const beluga_estimation::Matrix3d J_rpy = NumericalRpyJacobian(orientation);
  beluga_estimation::Matrix6d J = beluga_estimation::Matrix6d::Zero();
  J.topLeftCorner<3, 3>().setIdentity();
  J.bottomRightCorner<3, 3>() = J_rpy;
  const beluga_estimation::Matrix6d expected = J * rpy_covariance * J.transpose();

  EXPECT_TRUE(odom.GetPoseCovariance().isApprox(expected, 1e-6));
}

}  // namespace

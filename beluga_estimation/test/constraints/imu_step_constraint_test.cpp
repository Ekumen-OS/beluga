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

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/constraints/imu_step_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>

namespace {

TEST(ImuConstraint2_Test, ComputeLogLikelihood) {
  beluga_estimation::RawImuMeasurement2D<double> meas;
  meas.linear_acceleration = Eigen::Vector2d(1.0, 0.0);
  meas.angular_velocity = M_PI / 4.0;
  meas.linear_acceleration_covariance = Eigen::Matrix2d::Identity() * 0.1;
  meas.angular_velocity_variance = 0.1;

  beluga_estimation::TimePoint t0{};
  beluga_estimation::TimePoint t1 = t0 + std::chrono::seconds(1);
  beluga_estimation::ImuConstraint2_<double>::Variables vars{
      beluga_estimation::Identifier{"pose_k", t0}, beluga_estimation::Identifier{"vel_k", t0},
      beluga_estimation::Identifier{"pose_kp1", t1}, beluga_estimation::Identifier{"vel_kp1", t1},
      beluga_estimation::Identifier{"bias_k", t0}};

  beluga_estimation::ImuConstraint2_<double> constraint(
      beluga_estimation::Identifier{"imu_step_factor"}, std::move(vars), meas);

  // conditioning (k): pose, vel, bias
  beluga_estimation::VectorXd pose_k_vec = beluga_estimation::VectorXd::Zero(4);
  beluga_estimation::Map<beluga_estimation::SE2<double>>(pose_k_vec.data()) =
      beluga_estimation::SE2<double>(beluga_estimation::SO2<double>(0.0), Eigen::Vector2d(0.0, 0.0));
  beluga_estimation::VectorXd vel_k_vec = beluga_estimation::Vector2d(1.0, 0.0);
  beluga_estimation::VectorXd bias_k_vec = beluga_estimation::Vector3d::Zero();

  const std::vector<beluga_estimation::VectorXd> cond_values = {pose_k_vec, vel_k_vec, bias_k_vec};

  // dependent (k+1): perfect match
  beluga_estimation::VectorXd pose_kp1_vec = beluga_estimation::VectorXd::Zero(4);
  beluga_estimation::Map<beluga_estimation::SE2<double>>(pose_kp1_vec.data()) =
      beluga_estimation::SE2<double>(beluga_estimation::SO2<double>(M_PI / 4.0), Eigen::Vector2d(1.5, 0.0));
  beluga_estimation::VectorXd vel_kp1_vec = beluga_estimation::Vector2d(2.0, 0.0);

  const std::vector<beluga_estimation::VectorXd> dep_values = {pose_kp1_vec, vel_kp1_vec};

  double perfect_ll =
      constraint.ComputeLogLikelihood(beluga_estimation::to_view(dep_values), beluga_estimation::to_view(cond_values));

  // dependent (k+1): imperfect match in velocity
  beluga_estimation::VectorXd vel_kp1_err_vec = beluga_estimation::Vector2d(2.5, 0.0);
  const std::vector<beluga_estimation::VectorXd> dep_err_values = {pose_kp1_vec, vel_kp1_err_vec};

  double error_ll = constraint.ComputeLogLikelihood(
      beluga_estimation::to_view(dep_err_values), beluga_estimation::to_view(cond_values));

  EXPECT_GT(perfect_ll, error_ll);
}

TEST(ImuConstraint2_Test, CalcMean) {
  beluga_estimation::RawImuMeasurement2D<double> meas;
  meas.linear_acceleration = Eigen::Vector2d(1.0, 0.0);
  meas.angular_velocity = M_PI / 4.0;
  meas.linear_acceleration_covariance = Eigen::Matrix2d::Identity() * 0.1;
  meas.angular_velocity_variance = 0.1;

  beluga_estimation::TimePoint t0{};
  beluga_estimation::TimePoint t1 = t0 + std::chrono::seconds(1);
  beluga_estimation::ImuConstraint2_<double>::Variables vars{
      beluga_estimation::Identifier{"pose_k", t0}, beluga_estimation::Identifier{"vel_k", t0},
      beluga_estimation::Identifier{"pose_kp1", t1}, beluga_estimation::Identifier{"vel_kp1", t1},
      beluga_estimation::Identifier{"bias_k", t0}};

  beluga_estimation::ImuConstraint2_<double> constraint(
      beluga_estimation::Identifier{"imu_step_factor"}, std::move(vars), meas);

  beluga_estimation::VectorXd pose_k_vec = beluga_estimation::VectorXd::Zero(4);
  beluga_estimation::Map<beluga_estimation::SE2<double>>(pose_k_vec.data()) =
      beluga_estimation::SE2<double>(beluga_estimation::SO2<double>(0.0), Eigen::Vector2d(0.0, 0.0));
  beluga_estimation::VectorXd vel_k_vec = beluga_estimation::Vector2d(1.0, 0.0);
  beluga_estimation::VectorXd bias_k_vec = beluga_estimation::Vector3d::Zero();

  const std::vector<beluga_estimation::VectorXd> cond_values = {pose_k_vec, vel_k_vec, bias_k_vec};

  std::vector<double> mean_vec(6, 0.0);
  constraint.CalcMean(beluga_estimation::to_view(cond_values), beluga_estimation::to_view(mean_vec));

  beluga_estimation::Map<beluga_estimation::SE2<double>> pose_pred(mean_vec.data());
  beluga_estimation::Map<beluga_estimation::Vector2d> vel_pred(mean_vec.data() + 4);

  EXPECT_NEAR(pose_pred.translation().x(), 1.5, 1e-6);
  EXPECT_NEAR(pose_pred.translation().y(), 0.0, 1e-6);
  EXPECT_NEAR(pose_pred.so2().log(), M_PI / 4.0, 1e-6);
  EXPECT_NEAR(vel_pred.x(), 2.0, 1e-6);
  EXPECT_NEAR(vel_pred.y(), 0.0, 1e-6);
}

TEST(ImuConstraint2_Test, CalcCovariance) {
  beluga_estimation::RawImuMeasurement2D<double> meas;
  meas.linear_acceleration = Eigen::Vector2d(1.0, 0.0);
  meas.angular_velocity = M_PI / 4.0;
  meas.linear_acceleration_covariance = Eigen::Matrix2d::Identity() * 0.1;
  meas.angular_velocity_variance = 0.1;

  beluga_estimation::TimePoint t0{};
  beluga_estimation::TimePoint t1 = t0 + std::chrono::seconds(1);
  beluga_estimation::ImuConstraint2_<double>::Variables vars{
      beluga_estimation::Identifier{"pose_k", t0}, beluga_estimation::Identifier{"vel_k", t0},
      beluga_estimation::Identifier{"pose_kp1", t1}, beluga_estimation::Identifier{"vel_kp1", t1},
      beluga_estimation::Identifier{"bias_k", t0}};

  beluga_estimation::ImuConstraint2_<double> constraint(
      beluga_estimation::Identifier{"imu_step_factor"}, std::move(vars), meas);

  const std::vector<beluga_estimation::VectorXd> dummy_conditioning;

  beluga_estimation::Matrix5d actual_covariance;
  constraint.CalcCovariance(
      beluga_estimation::to_view(dummy_conditioning), beluga_estimation::to_view(actual_covariance));

  // Verify that the diagonal elements are greater than zero
  for (int i = 0; i < 5; ++i) {
    EXPECT_GT(actual_covariance(i, i), 0.0);
  }
}

TEST(ImuConstraint2_Test, CalcResidual) {
  beluga_estimation::RawImuMeasurement2D<double> meas;
  meas.linear_acceleration = Eigen::Vector2d(1.0, 0.0);
  meas.angular_velocity = M_PI / 4.0;
  meas.linear_acceleration_covariance = Eigen::Matrix2d::Identity() * 0.1;
  meas.angular_velocity_variance = 0.1;

  beluga_estimation::TimePoint t0{};
  beluga_estimation::TimePoint t1 = t0 + std::chrono::seconds(1);
  beluga_estimation::ImuConstraint2_<double>::Variables vars{
      beluga_estimation::Identifier{"pose_k", t0}, beluga_estimation::Identifier{"vel_k", t0},
      beluga_estimation::Identifier{"pose_kp1", t1}, beluga_estimation::Identifier{"vel_kp1", t1},
      beluga_estimation::Identifier{"bias_k", t0}};

  beluga_estimation::ImuConstraint2_<double> constraint(
      beluga_estimation::Identifier{"imu_step_factor"}, std::move(vars), meas);

  beluga_estimation::VectorXd pose_k_vec = beluga_estimation::VectorXd::Zero(4);
  beluga_estimation::Map<beluga_estimation::SE2<double>>(pose_k_vec.data()) =
      beluga_estimation::SE2<double>(beluga_estimation::SO2<double>(0.0), Eigen::Vector2d(0.0, 0.0));
  beluga_estimation::VectorXd vel_k_vec = beluga_estimation::Vector2d(1.0, 0.0);
  beluga_estimation::VectorXd bias_k_vec = beluga_estimation::Vector3d::Zero();

  const std::vector<beluga_estimation::VectorXd> cond_values = {pose_k_vec, vel_k_vec, bias_k_vec};

  beluga_estimation::VectorXd pose_kp1_vec = beluga_estimation::VectorXd::Zero(4);
  beluga_estimation::Map<beluga_estimation::SE2<double>>(pose_kp1_vec.data()) =
      beluga_estimation::SE2<double>(beluga_estimation::SO2<double>(M_PI / 4.0), Eigen::Vector2d(1.5, 0.0));
  beluga_estimation::VectorXd vel_kp1_vec = beluga_estimation::Vector2d(2.0, 0.0);

  const std::vector<beluga_estimation::VectorXd> dep_values = {pose_kp1_vec, vel_kp1_vec};

  beluga_estimation::Vector5d actual_residual;

  constraint.CalcResidual(
      beluga_estimation::to_view(dep_values), beluga_estimation::to_view(cond_values),
      beluga_estimation::to_view(actual_residual));

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(actual_residual.size()));

  // With a perfect match, the residual should be near zero.
  for (int i = 0; i < 5; ++i) {
    EXPECT_NEAR(actual_residual[i], 0.0, 1e-6);
  }

  // Introduce an error in velocity
  beluga_estimation::VectorXd vel_kp1_err_vec = beluga_estimation::Vector2d(2.5, 0.0);
  const std::vector<beluga_estimation::VectorXd> dep_err_values = {pose_kp1_vec, vel_kp1_err_vec};

  constraint.CalcResidual(
      beluga_estimation::to_view(dep_err_values), beluga_estimation::to_view(cond_values),
      beluga_estimation::to_view(actual_residual));

  // The velocity error should be exactly 0.5 in the x component
  EXPECT_NEAR(actual_residual[3], 0.5, 1e-6);
}

TEST(ImuConstraint2_Test, OptionalBias) {
  beluga_estimation::RawImuMeasurement2D<double> meas;
  meas.linear_acceleration = Eigen::Vector2d(1.0, 0.0);
  meas.angular_velocity = M_PI / 4.0;
  meas.linear_acceleration_covariance = Eigen::Matrix2d::Identity() * 0.1;
  meas.angular_velocity_variance = 0.1;

  beluga_estimation::TimePoint t0{};
  beluga_estimation::TimePoint t1 = t0 + std::chrono::seconds(1);
  // Setup constraint specifically with only 2 conditioning variables (no bias tracking)
  beluga_estimation::ImuConstraint2_<double>::Variables vars{
      beluga_estimation::Identifier{"pose_k", t0}, beluga_estimation::Identifier{"vel_k", t0},
      beluga_estimation::Identifier{"pose_kp1", t1}, beluga_estimation::Identifier{"vel_kp1", t1}};

  beluga_estimation::ImuConstraint2_<double> constraint(
      beluga_estimation::Identifier{"imu_step_factor"}, std::move(vars), meas);

  beluga_estimation::VectorXd pose_k_vec = beluga_estimation::VectorXd::Zero(4);
  beluga_estimation::Map<beluga_estimation::SE2<double>>(pose_k_vec.data()) =
      beluga_estimation::SE2<double>(beluga_estimation::SO2<double>(0.0), Eigen::Vector2d(0.0, 0.0));
  beluga_estimation::VectorXd vel_k_vec = beluga_estimation::Vector2d(1.0, 0.0);

  const std::vector<beluga_estimation::VectorXd> cond_values = {pose_k_vec, vel_k_vec};

  std::vector<double> mean_vec(6, 0.0);
  constraint.CalcMean(beluga_estimation::to_view(cond_values), beluga_estimation::to_view(mean_vec));

  beluga_estimation::Map<beluga_estimation::SE2<double>> pose_pred(mean_vec.data());
  beluga_estimation::Map<beluga_estimation::Vector2d> vel_pred(mean_vec.data() + 4);

  // Expected same behavior as if bias was explicitly evaluated at absolute zeros
  EXPECT_NEAR(pose_pred.translation().x(), 1.5, 1e-6);
  EXPECT_NEAR(pose_pred.translation().y(), 0.0, 1e-6);
  EXPECT_NEAR(pose_pred.so2().log(), M_PI / 4.0, 1e-6);
  EXPECT_NEAR(vel_pred.x(), 2.0, 1e-6);
  EXPECT_NEAR(vel_pred.y(), 0.0, 1e-6);
}

TEST(ImuConstraint2_Test, CalcMeanJacobian) {
  beluga_estimation::RawImuMeasurement2D<double> meas;
  meas.linear_acceleration = Eigen::Vector2d(1.0, 0.0);
  meas.angular_velocity = M_PI / 4.0;
  meas.linear_acceleration_covariance = Eigen::Matrix2d::Identity() * 0.1;
  meas.angular_velocity_variance = 0.1;

  beluga_estimation::TimePoint t0{};
  beluga_estimation::TimePoint t1 = t0 + std::chrono::seconds(1);
  beluga_estimation::ImuConstraint2_<double>::Variables vars{
      beluga_estimation::Identifier{"pose_k", t0}, beluga_estimation::Identifier{"vel_k", t0},
      beluga_estimation::Identifier{"pose_kp1", t1}, beluga_estimation::Identifier{"vel_kp1", t1},
      beluga_estimation::Identifier{"bias_k", t0}};

  beluga_estimation::ImuConstraint2_<double> constraint(
      beluga_estimation::Identifier{"imu_step_factor"}, std::move(vars), meas);

  beluga_estimation::SE2<double> pose_k(beluga_estimation::SO2<double>(0.0), Eigen::Vector2d(0.0, 0.0));
  Eigen::Vector2d vel_k(1.0, 0.0);
  Eigen::Vector3d bias_k(0.0, 0.0, 0.0);

  beluga_estimation::SE2<double> pose_pred;
  Eigen::Vector2d vel_pred;
  beluga_estimation::Matrix<double, 5, Eigen::Dynamic> jacobian(5, 8);

  constraint.CalcMean(pose_k, vel_k, bias_k, pose_pred, vel_pred, jacobian);

  // Velocity prediction w.r.t velocity state is an identity block
  EXPECT_DOUBLE_EQ(jacobian(3, 3), 1.0);
  EXPECT_DOUBLE_EQ(jacobian(4, 4), 1.0);

  // Theta prediction w.r.t bias omega is -dt
  EXPECT_DOUBLE_EQ(jacobian(2, 5), -1.0);
}

TEST(ImuConstraint2_Test, CalcResidualJacobian) {
  beluga_estimation::RawImuMeasurement2D<double> meas;
  meas.linear_acceleration = Eigen::Vector2d(1.0, 0.0);
  meas.angular_velocity = M_PI / 4.0;
  meas.linear_acceleration_covariance = Eigen::Matrix2d::Identity() * 0.1;
  meas.angular_velocity_variance = 0.1;

  beluga_estimation::TimePoint t0{};
  beluga_estimation::TimePoint t1 = t0 + std::chrono::seconds(1);
  beluga_estimation::ImuConstraint2_<double>::Variables vars{
      beluga_estimation::Identifier{"pose_k", t0}, beluga_estimation::Identifier{"vel_k", t0},
      beluga_estimation::Identifier{"pose_kp1", t1}, beluga_estimation::Identifier{"vel_kp1", t1},
      beluga_estimation::Identifier{"bias_k", t0}};

  beluga_estimation::ImuConstraint2_<double> constraint(
      beluga_estimation::Identifier{"imu_step_factor"}, std::move(vars), meas);

  beluga_estimation::SE2<double> pose_k(beluga_estimation::SO2<double>(0.0), Eigen::Vector2d(0.0, 0.0));
  Eigen::Vector2d vel_k(1.0, 0.0);
  Eigen::Vector3d bias_k(0.0, 0.0, 0.0);

  beluga_estimation::SE2<double> pose_kp1(beluga_estimation::SO2<double>(M_PI / 4.0), Eigen::Vector2d(1.5, 0.0));
  Eigen::Vector2d vel_kp1(2.0, 0.0);

  beluga_estimation::Vector5d residual;
  beluga_estimation::Matrix5d jacobian;

  constraint.CalcResidual(pose_kp1, vel_kp1, pose_k, vel_k, bias_k, residual, jacobian);

  // The residual dimension must match the constraint's residual size.
  EXPECT_EQ(constraint.ResidualSize(), static_cast<std::size_t>(residual.size()));

  // Diagonal blocks for pose and velocity should generally start with identity
  EXPECT_DOUBLE_EQ(jacobian(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(jacobian(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(jacobian(2, 2), 1.0);
  EXPECT_DOUBLE_EQ(jacobian(3, 3), 1.0);
  EXPECT_DOUBLE_EQ(jacobian(4, 4), 1.0);
}

TEST(ImuConstraint2_Test, RejectsMismatchedVariableDimensions) {
  beluga_estimation::RawImuMeasurement2D<double> meas;
  meas.linear_acceleration = Eigen::Vector2d(1.0, 0.0);
  meas.angular_velocity = M_PI / 4.0;
  meas.linear_acceleration_covariance = Eigen::Matrix2d::Identity() * 0.1;
  meas.angular_velocity_variance = 0.1;

  beluga_estimation::TimePoint t0{};
  beluga_estimation::TimePoint t1 = t0 + std::chrono::seconds(1);
  beluga_estimation::ImuConstraint2_<double>::Variables vars{
      beluga_estimation::Identifier{"pose_k", t0}, beluga_estimation::Identifier{"vel_k", t0},
      beluga_estimation::Identifier{"pose_kp1", t1}, beluga_estimation::Identifier{"vel_kp1", t1},
      beluga_estimation::Identifier{"bias_k", t0}};

  beluga_estimation::ImuConstraint2_<double> constraint(
      beluga_estimation::Identifier{"imu_step_factor"}, std::move(vars), meas);

  beluga_estimation::VectorXd pose = beluga_estimation::VectorXd::Zero(4);
  beluga_estimation::Map<beluga_estimation::SE2<double>>(pose.data()) =
      beluga_estimation::SE2<double>(beluga_estimation::SO2<double>(0.0), beluga_estimation::Vector2d::Zero());
  const beluga_estimation::VectorXd vel = beluga_estimation::Vector2d(1.0, 0.0);
  const beluga_estimation::VectorXd bias = beluga_estimation::Vector3d::Zero();
  const beluga_estimation::VectorXd bad_dimension = beluga_estimation::VectorXd::Zero(3);

  // Euclidean variables are dynamically dimensioned, so their extents are
  // preconditions that must be demanded, not invariants.
  {
    const std::vector<beluga_estimation::VectorXd> dep_values = {pose, bad_dimension};
    const std::vector<beluga_estimation::VectorXd> cond_values = {pose, vel, bias};
    beluga_estimation::Vector5d residual;
    EXPECT_THROW(
        constraint.CalcResidual(
            beluga_estimation::to_view(dep_values), beluga_estimation::to_view(cond_values),
            beluga_estimation::to_view(residual)),
        std::runtime_error);
  }
  {
    const std::vector<beluga_estimation::VectorXd> cond_values = {pose, bad_dimension, bias};
    beluga_estimation::VectorXd mean = beluga_estimation::VectorXd::Zero(6);
    EXPECT_THROW(
        constraint.CalcMean(beluga_estimation::to_view(cond_values), beluga_estimation::to_view(mean)),
        std::runtime_error);
  }
  {
    const std::vector<beluga_estimation::VectorXd> dep_values = {pose, vel};
    const beluga_estimation::VectorXd bad_bias = beluga_estimation::VectorXd::Zero(2);
    const std::vector<beluga_estimation::VectorXd> cond_values = {pose, vel, bad_bias};
    EXPECT_THROW(
        constraint.ComputeLogLikelihood(
            beluga_estimation::to_view(dep_values), beluga_estimation::to_view(cond_values)),
        std::runtime_error);
  }
}

}  // namespace

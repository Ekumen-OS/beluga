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

#include <cassert>
#include <chrono>
#include <cmath>
#include <optional>
#include <random>
#include <vector>

#include <Eigen/Core>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/locally_normal_constraint.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief Raw IMU measurement for a single step 2D kinematic constraint.
 *
 * Encapsulates the raw linear acceleration, angular velocity, and a dense 5x5
 * covariance matrix for the noise over the integration period dt.
 * @tparam T The scalar type.
 */
template <typename T>
struct RawImuMeasurement2D {
  // Raw linear acceleration [ax, ay]
  beluga_estimation::Vector2<T> linear_acceleration;

  // Raw angular velocity (omega around Z-axis)
  T angular_velocity;

  // Covariance of the linear acceleration measurement (2x2)
  beluga_estimation::Matrix2<T> linear_acceleration_covariance;

  // Variance of the angular velocity measurement
  T angular_velocity_variance;
};

/**
 * @brief Step-by-step IMU-driven kinematic constraint for Pose and Velocity in 2D.
 *
 * Enforces the relationship between (x_k, v_k, b_k) and (x_{k+1}, v_{k+1}) using
 * a single raw IMU measurement (not pre-integrated).
 * Computes a joint 5-dimensional residual on the SE(2) x R^2 manifold.
 *
 * State expectations in spans (for both dependent and conditioning):
 * - Index 0: Pose SE(2) (size 4: cos, sin, x, y)
 * - Index 1: Velocity R^2 (size 2: vx, vy)
 * - Index 2: IMU Bias R^3 (size 3: b_omega, b_ax, b_ay) - only needed in conditioning (optional)
 */
template <typename T>
class ImuConstraint2_ : public LocallyNormalConstraint_<T> {
 public:
  /**
   * @brief The specific set of variable identifiers involved in this constraint.
   */
  struct Variables {
    Identifier pose_k;                                ///< Conditioning variable for the source pose.
    Identifier vel_k;                                 ///< Conditioning variable for the source velocity.
    Identifier pose_kp1;                              ///< Dependent variable for the target pose.
    Identifier vel_kp1;                               ///< Dependent variable for the target velocity.
    std::optional<Identifier> bias_k = std::nullopt;  ///< Optional conditioning variable for the IMU bias.
  };

  ImuConstraint2_(Identifier id, Variables variables, RawImuMeasurement2D<T> measurement)
      : LocallyNormalConstraint_<T>(std::move(id)),
        measurement_(std::move(measurement)),
        dt_(CalculateDt(variables)),
        error_distribution_(beluga_estimation::Vector5<T>::Zero(), ComputeCovariance(measurement_, dt_)) {
    dependent_ids_ = {std::move(variables.pose_kp1), std::move(variables.vel_kp1)};
    conditioning_ids_ = {std::move(variables.pose_k), std::move(variables.vel_k)};
    if (variables.bias_k) {
      conditioning_ids_.push_back(std::move(*variables.bias_k));
    }
  }

  bool HasDependentVariables() const override { return true; }
  bool HasConditioningVariables() const override { return true; }

  std_future::span<const Identifier> DependentVariables() const override { return dependent_ids_; }
  std_future::span<const Identifier> ConditioningVariables() const override { return conditioning_ids_; }

  [[nodiscard]] std::size_t ResidualSize() const override {
    // Pose and linear velocity R^2 residuals
    return beluga_estimation::SE2<T>::DoF + 2;
  }

  T ComputeLogLikelihood(
      view<std_future::span<const T>> dependent_values,
      view<std_future::span<const T>> conditioning_values) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_values.size() >= 2);
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() >= 2);
    BELUGA_ESTIMATION_ASSERT(dependent_values[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(dependent_values[1].size() == 2, "velocity variable must have dimension 2");
    BELUGA_ESTIMATION_ASSERT(conditioning_values[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(conditioning_values[1].size() == 2, "velocity variable must have dimension 2");

    beluga_estimation::Vector5<T> error_vector;

    std::optional<beluga_estimation::Ref<const beluga_estimation::Vector3<T>>> bias_opt = std::nullopt;
    if (conditioning_values.size() >= 3) {
      BELUGA_ESTIMATION_DEMAND(conditioning_values[2].size() == 3, "bias variable must have dimension 3");
      bias_opt.emplace(beluga_estimation::Map<const beluga_estimation::Vector3<T>>(conditioning_values[2].data()));
    }

    CalcResidual(
        beluga_estimation::Map<const beluga_estimation::SE2<T>>(dependent_values[0].data()),
        beluga_estimation::Map<const beluga_estimation::Vector2<T>>(dependent_values[1].data()),
        beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_values[0].data()),
        beluga_estimation::Map<const beluga_estimation::Vector2<T>>(conditioning_values[1].data()), bias_opt,
        error_vector);

    return error_distribution_.LogProbability(error_vector);
  }

  void TakeSamples(
      view<std_future::span<const T>> conditioning_values,
      view<view<std_future::span<T>>> sampled_dependent_values,
      RandomNumberGenerator& generator) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() >= 2);
    BELUGA_ESTIMATION_ASSERT(sampled_dependent_values.size() >= 2);
    BELUGA_ESTIMATION_ASSERT(conditioning_values[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(conditioning_values[1].size() == 2, "velocity variable must have dimension 2");

    beluga_estimation::Map<const beluga_estimation::SE2<T>> pose_k(conditioning_values[0].data());
    beluga_estimation::Map<const beluga_estimation::Vector2<T>> vel_k(conditioning_values[1].data());

    std::optional<beluga_estimation::Ref<const beluga_estimation::Vector3<T>>> bias_opt = std::nullopt;
    if (conditioning_values.size() >= 3) {
      BELUGA_ESTIMATION_DEMAND(conditioning_values[2].size() == 3, "bias variable must have dimension 3");
      bias_opt.emplace(beluga_estimation::Map<const beluga_estimation::Vector3<T>>(conditioning_values[2].data()));
    }

    beluga_estimation::SE2<T> pose_pred;
    beluga_estimation::Vector2<T> vel_pred;
    CalcMean(pose_k, vel_k, bias_opt, pose_pred, vel_pred);

    auto sampled_poses = sampled_dependent_values[0];
    auto sampled_vels = sampled_dependent_values[1];
    BELUGA_ESTIMATION_ASSERT(sampled_poses.size() == sampled_vels.size());

    std::vector<beluga_estimation::Vector5<T>> d_samples = error_distribution_.Sample(sampled_poses.size(), generator);

    for (size_t i = 0; i < sampled_poses.size(); ++i) {
      auto pose_out = sampled_poses[i];
      auto vel_out = sampled_vels[i];
      BELUGA_ESTIMATION_ASSERT(pose_out.size() == 4);
      BELUGA_ESTIMATION_ASSERT(vel_out.size() == 2);

      const beluga_estimation::Vector5<T>& d = d_samples[i];

      beluga_estimation::Vector3<T> d_pose = d.template head<3>();
      beluga_estimation::Vector2<T> d_vel = d.template tail<2>();

      beluga_estimation::SE2<T> pose_sample = pose_pred * beluga_estimation::SE2<T>::exp(d_pose);
      beluga_estimation::Vector2<T> vel_sample = vel_pred + pose_k.so2() * d_vel;

      beluga_estimation::Map<beluga_estimation::SE2<T>>(pose_out.data()) = pose_sample;
      beluga_estimation::Map<beluga_estimation::Vector2<T>>(vel_out.data()) = vel_sample;
    }
  }

  /**
   * @brief Calculates the mean prediction (typed interface).
   */
  void CalcMean(
      beluga_estimation::Ref<const beluga_estimation::SE2<T>> pose_k,
      beluga_estimation::Ref<const beluga_estimation::Vector2<T>> vel_k,
      std::optional<beluga_estimation::Ref<const beluga_estimation::Vector3<T>>> bias_k,
      beluga_estimation::Ref<beluga_estimation::SE2<T>> pose_pred,
      beluga_estimation::Ref<beluga_estimation::Vector2<T>> vel_pred,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix<T, 5, Eigen::Dynamic>>> jacobian = std::nullopt,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix<T, 5, Eigen::Dynamic>>> hessian =
          std::nullopt) const {
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");

    T b_omega = 0.0;
    beluga_estimation::Vector2<T> b_a = beluga_estimation::Vector2<T>::Zero();
    if (bias_k) {
      b_omega = (*bias_k)(0);
      b_a = bias_k->template tail<2>();
    }

    T corrected_omega = measurement_.angular_velocity - b_omega;
    beluga_estimation::Vector2<T> corrected_a = measurement_.linear_acceleration - b_a;

    beluga_estimation::SE2<T> delta_p_meas;
    delta_p_meas.so2() = beluga_estimation::SO2<T>::exp(corrected_omega * dt_);
    delta_p_meas.translation() = static_cast<T>(0.5) * corrected_a * dt_ * dt_;

    beluga_estimation::Vector2<T> delta_v_meas = corrected_a * dt_;

    pose_pred.so2() = pose_k.so2() * delta_p_meas.so2();
    pose_pred.translation() = pose_k.translation() + vel_k * dt_ + pose_k.so2() * delta_p_meas.translation();

    vel_pred = vel_k + pose_k.so2() * delta_v_meas;

    if (jacobian) {
      auto& J = *jacobian;
      J.setZero();

      J.template block<3, 3>(0, 0) = delta_p_meas.inverse().Adj();
      beluga_estimation::Vector2<T> J_dv(-delta_v_meas.y(), delta_v_meas.x());
      J.template block<2, 1>(3, 2) = pose_k.so2().matrix() * J_dv;

      J.template block<2, 2>(0, 3) = pose_pred.so2().inverse().matrix() * dt_;
      J.template block<2, 2>(3, 3).setIdentity();

      if (bias_k) {
        J(2, 5) = -dt_;
        J.template block<2, 2>(0, 6) = -static_cast<T>(0.5) * dt_ * dt_ * delta_p_meas.so2().inverse().matrix();
        J.template block<2, 2>(3, 6) = -pose_k.so2().matrix() * dt_;
      }
    }
  }

  /**
   * @brief Calculates the expected mean of the dependent variables (span interface).
   */
  void CalcMean(
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> mean,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() >= 2);
    BELUGA_ESTIMATION_ASSERT(mean.size() >= 6);
    BELUGA_ESTIMATION_ASSERT(conditioning_variables[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(conditioning_variables[1].size() == 2, "velocity variable must have dimension 2");
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");

    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix<T, 5, Eigen::Dynamic>>> jacobian_opt;
    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix<T, 5, Eigen::Dynamic>>> hessian_opt;

    if (jacobian) {
      jacobian_opt.emplace(beluga_estimation::Map<beluga_estimation::Matrix<T, 5, Eigen::Dynamic>>(*jacobian));
    }
    if (hessian) {
      hessian_opt.emplace(beluga_estimation::Map<beluga_estimation::Matrix<T, 5, Eigen::Dynamic>>(*hessian));
    }

    auto pose_pred = beluga_estimation::Map<beluga_estimation::SE2<T>>(mean.data());
    auto vel_pred = beluga_estimation::Map<beluga_estimation::Vector2<T>>(mean.data() + 4);

    std::optional<beluga_estimation::Ref<const beluga_estimation::Vector3<T>>> bias_opt = std::nullopt;
    if (conditioning_variables.size() >= 3) {
      BELUGA_ESTIMATION_DEMAND(conditioning_variables[2].size() == 3, "bias variable must have dimension 3");
      bias_opt.emplace(beluga_estimation::Map<const beluga_estimation::Vector3<T>>(conditioning_variables[2].data()));
    }

    CalcMean(
        beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_variables[0].data()),
        beluga_estimation::Map<const beluga_estimation::Vector2<T>>(conditioning_variables[1].data()), bias_opt,
        pose_pred, vel_pred, jacobian_opt, hessian_opt);
  }

  /**
   * @brief Provides the tangent-space covariance (typed interface).
   */
  void CalcCovariance(beluga_estimation::Ref<beluga_estimation::Matrix5<T>> covariance) const {
    covariance = error_distribution_.Covariance();
  }

  /**
   * @brief Provides the tangent-space covariance (span interface).
   */
  void CalcCovariance(
      [[maybe_unused]] view<std_future::span<const T>> conditioning_variables,
      std_future::mdspan<T, 2> covariance) const override {
    auto cov_map = beluga_estimation::Map<beluga_estimation::Matrix5<T>>(covariance);
    CalcCovariance(cov_map);
  }

  /**
   * @brief Calculates the residual and its Jacobian in the tangent space (typed interface).
   */
  void CalcResidual(
      beluga_estimation::Ref<const beluga_estimation::SE2<T>> pose_kp1,
      beluga_estimation::Ref<const beluga_estimation::Vector2<T>> vel_kp1,
      beluga_estimation::Ref<const beluga_estimation::SE2<T>> pose_k,
      beluga_estimation::Ref<const beluga_estimation::Vector2<T>> vel_k,
      std::optional<beluga_estimation::Ref<const beluga_estimation::Vector3<T>>> bias_k,
      beluga_estimation::Ref<beluga_estimation::Vector5<T>> residual,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix5<T>>> jacobian = std::nullopt,
      std::optional<beluga_estimation::Ref<beluga_estimation::Matrix5<T>>> hessian = std::nullopt) const {
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");

    beluga_estimation::SE2<T> pose_pred;
    beluga_estimation::Vector2<T> vel_pred;
    CalcMean(pose_k, vel_k, bias_k, pose_pred, vel_pred);

    beluga_estimation::SE2<T> error_se2 = pose_pred.inverse() * pose_kp1;
    beluga_estimation::Vector3<T> error_pose = error_se2.log();

    beluga_estimation::Vector2<T> error_vel = pose_k.so2().inverse() * (vel_kp1 - vel_pred);

    residual << error_pose, error_vel;

    if (jacobian) {
      auto& J = *jacobian;
      J.setZero();

      J.template block<3, 3>(0, 0).setIdentity();
      const T half_theta = residual(2) / static_cast<T>(2.0);
      J(0, 1) = -half_theta;
      J(1, 0) = half_theta;
      J(0, 2) = residual(1) / static_cast<T>(2.0);
      J(1, 2) = -residual(0) / static_cast<T>(2.0);

      J.template block<2, 2>(3, 3) = pose_k.so2().inverse().matrix();
    }
  }

  /**
   * @brief Calculates the residual and its Jacobian in the tangent space (span interface).
   */
  void CalcResidual(
      view<std_future::span<const T>> dependent_variables,
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> residual,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_variables.size() >= 2);
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() >= 2);
    BELUGA_ESTIMATION_ASSERT(residual.size() >= 5);
    BELUGA_ESTIMATION_ASSERT(dependent_variables[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(dependent_variables[1].size() == 2, "velocity variable must have dimension 2");
    BELUGA_ESTIMATION_ASSERT(conditioning_variables[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(conditioning_variables[1].size() == 2, "velocity variable must have dimension 2");
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");

    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix5<T>>> jacobian_opt;
    std::optional<beluga_estimation::Ref<beluga_estimation::Matrix5<T>>> hessian_opt;

    if (jacobian) {
      jacobian_opt.emplace(beluga_estimation::Map<beluga_estimation::Matrix5<T>>(*jacobian));
    }
    if (hessian) {
      hessian_opt.emplace(beluga_estimation::Map<beluga_estimation::Matrix5<T>>(*hessian));
    }

    std::optional<beluga_estimation::Ref<const beluga_estimation::Vector3<T>>> bias_opt = std::nullopt;
    if (conditioning_variables.size() >= 3) {
      BELUGA_ESTIMATION_DEMAND(conditioning_variables[2].size() == 3, "bias variable must have dimension 3");
      bias_opt.emplace(beluga_estimation::Map<const beluga_estimation::Vector3<T>>(conditioning_variables[2].data()));
    }

    CalcResidual(
        beluga_estimation::Map<const beluga_estimation::SE2<T>>(dependent_variables[0].data()),
        beluga_estimation::Map<const beluga_estimation::Vector2<T>>(dependent_variables[1].data()),
        beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_variables[0].data()),
        beluga_estimation::Map<const beluga_estimation::Vector2<T>>(conditioning_variables[1].data()), bias_opt,
        beluga_estimation::Map<beluga_estimation::Vector5<T>>(residual.data()), jacobian_opt, hessian_opt);
  }

 private:
  static T CalculateDt(const Variables& vars) {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    return duration_cast<duration<T>>(vars.pose_kp1.timestamp - vars.pose_k.timestamp).count();
  }

  /**
   * @brief Computes the 5x5 covariance matrix for the step constraint via first-order error propagation.
   *
   * This propagates the raw IMU measurement noise into a zero-mean Gaussian error defined directly
   * on the tangent space of the combined SE(2) x R^2 manifold. The approach is inspired by the IMU
   * preintegration theory described in:
   *
   * Forster, C., Carlone, L., Dellaert, F., & Scaramuzza, D. (2015). "IMU Preintegration on Manifold
   * for Efficient Visual-Inertial Maximum-a-Posteriori Estimation".
   *
   * \b Mathematical \b Formulation:
   * 1. \b Kinematics:
   *    \f[ R_{k+1} = R_k \text{Exp}((\tilde{\omega} - b_\omega) dt) \f]
   *    \f[ v_{k+1} = v_k + R_k (\tilde{a} - b_a) dt \f]
   *    \f[ p_{k+1} = p_k + v_k dt + \frac{1}{2} R_k (\tilde{a} - b_a) dt^2 \f]
   *
   * 2. \b First-order \b propagation:
   *    We map the 3D measurement noise covariance \f$ \Sigma_{\text{imu}} \f$ associated with
   *    \f$ [\tilde{a}_x, \tilde{a}_y, \tilde{\omega}]^T \f$ into the 5D tangent space
   *    \f$ \mathfrak{se}(2) \times \mathbb{R}^2 \f$ representing the pose and velocity error.
   *    \f[ \Sigma_{\text{step}} \approx J \Sigma_{\text{imu}} J^T \f]
   *
   * 3. \b Jacobian \b \f$ J \f$:
   *    Under a small-angle approximation during the integration period \f$ dt \f$, we assume the
   *    local frame remains roughly constant. This decouples the rotational and translational noise
   *    components linearly:
   *    \f[
   *    J = \frac{\partial (\Delta \text{state})}{\partial \text{meas}} =
   *    \begin{bmatrix}
   *      \frac{1}{2} dt^2 & 0 & 0 \\
   *      0 & \frac{1}{2} dt^2 & 0 \\
   *      0 & 0 & dt \\
   *      dt & 0 & 0 \\
   *      0 & dt & 0
   *    \end{bmatrix}
   *    \f]
   *
   * \b Tradeoffs:
   * - This employs a small-angle approximation. If \f$ \tilde{\omega} dt \f$ is large, mixing the
   *   local frame acceleration directly into the global position/velocity increment without
   *   accounting for rotation noise cross-terms introduces inaccuracies.
   * - However, for high-rate IMUs where \f$ dt \f$ is very small, this decoupled first-order
   *   approximation is highly efficient and practically accurate.
   *
   * @param measurement The raw IMU measurement containing the initial variances.
   * @param dt The integration time step.
   * @return The 5x5 covariance matrix representing the combined pose and velocity uncertainty.
   */
  static beluga_estimation::Matrix5<T> ComputeCovariance(const RawImuMeasurement2D<T>& measurement, T dt) {
    // Construct the 5x5 covariance matrix internally via error propagation
    beluga_estimation::Matrix3<T> cov_meas = beluga_estimation::Matrix3<T>::Zero();
    cov_meas.template block<2, 2>(0, 0) = measurement.linear_acceleration_covariance;
    cov_meas(2, 2) = measurement.angular_velocity_variance;

    // Jacobian mapping [a_x, a_y, omega] to [pose (3), vel (2)]
    // Note: Sophus SE2 log maps to [x, y, theta] or [theta, x, y] depending on the version.
    // In Beluga 2D, the tangent vector is typically [x, y, theta].
    beluga_estimation::Matrix<T, 5, 3> J = beluga_estimation::Matrix<T, 5, 3>::Zero();
    T dt2 = dt * dt;

    // d(pose_x)/d(a_x) = 0.5 * dt^2, d(pose_y)/d(a_y) = 0.5 * dt^2
    J(0, 0) = static_cast<T>(0.5) * dt2;
    J(1, 1) = static_cast<T>(0.5) * dt2;
    // d(pose_theta)/d(omega) = dt
    J(2, 2) = dt;
    // d(vel_x)/d(a_x) = dt, d(vel_y)/d(a_y) = dt
    J(3, 0) = dt;
    J(4, 1) = dt;

    beluga_estimation::Matrix5<T> cov_5d = J * cov_meas * J.transpose();

    // Add small diagonal to ensure positive definiteness
    cov_5d += beluga_estimation::Matrix5<T>::Identity() * static_cast<T>(1e-9);

    return cov_5d;
  }

  RawImuMeasurement2D<T> measurement_;
  T dt_;
  beluga_estimation::NormalDistribution_<T, 5> error_distribution_;
  std::vector<Identifier> dependent_ids_;
  std::vector<Identifier> conditioning_ids_;
};

}  // namespace beluga_estimation

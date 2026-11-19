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

#include <array>
#include <cassert>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <utility>

#include <Eigen/Cholesky>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/probability.hpp>
#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/core/locally_normal_constraint.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/distributions/se2_normal_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief Constant velocity kinematic constraint p(x_j, v_j | x_i, v_i).
 *
 * Models the transition:
 *   x_j = x_i * exp(v_i * dt) + pose noise
 *   v_j ≈ v_i                 + velocity noise
 *
 * Process noise is specified as two separate blocks: Q_pose (3x3) and Q_vel (3x3), modeled as
 * independent, so the joint log-likelihood factors as log p(pose_j) + log p(vel_j).
 *
 * @tparam T Scalar type (float or double).
 */
template <typename T>
class SE2ConstantVelocityKinematicConstraint_ : public LocallyNormalConstraint_<T> {
 public:
  SE2ConstantVelocityKinematicConstraint_(
      Identifier id,
      Identifier pose_i_id,
      Identifier vel_i_id,
      Identifier pose_j_id,
      Identifier vel_j_id,
      beluga_estimation::Matrix3<T> Q_pose,
      beluga_estimation::Matrix3<T> Q_vel)
      : LocallyNormalConstraint_<T>(std::move(id)),
        conditioning_ids_{std::move(pose_i_id), std::move(vel_i_id)},
        dependent_ids_{std::move(pose_j_id), std::move(vel_j_id)},
        dt_(CalculateDt(PoseIId(), PoseJId())),
        Q_pose_(std::move(Q_pose)),
        Q_vel_(std::move(Q_vel)) {
    BELUGA_ESTIMATION_DEMAND(dt_ > T{0}, "dt must be positive");
    BELUGA_ESTIMATION_DEMAND(IsCovarianceMatrix(Q_pose_), "Q_pose must be symmetric positive definite");
    BELUGA_ESTIMATION_DEMAND(IsCovarianceMatrix(Q_vel_), "Q_vel must be symmetric positive definite");
  }

  bool HasDependentVariables() const override { return true; }
  bool HasConditioningVariables() const override { return true; }

  std_future::span<const Identifier> DependentVariables() const override {
    return std_future::span<const Identifier>(dependent_ids_.data(), dependent_ids_.size());
  }

  std_future::span<const Identifier> ConditioningVariables() const override {
    return std_future::span<const Identifier>(conditioning_ids_.data(), conditioning_ids_.size());
  }

  [[nodiscard]] std::size_t ResidualSize() const override {
    // Pose + velocity residuals, same dimensionality.
    return 2 * beluga_estimation::SE2<T>::DoF;
  }

  T ComputeLogLikelihood(
      const beluga_estimation::SE2<T>& pose_j,
      const beluga_estimation::Vector3<T>& vel_j,
      const beluga_estimation::SE2<T>& pose_i,
      const beluga_estimation::Vector3<T>& vel_i) const {
    const beluga_estimation::SE2<T> expected_pose = pose_i * beluga_estimation::SE2<T>::exp(vel_i * dt_);
    SE2NormalDistribution_<T> pose_dist(expected_pose, Q_pose_);
    NormalDistribution_<T, 3> vel_dist(vel_i, Q_vel_);
    return pose_dist.LogProbability(std_future::span<const T>{pose_j.data(), 4}) +
           vel_dist.LogProbability(std_future::span<const T>{vel_j.data(), 3});
  }

  void TakeSamples(
      const beluga_estimation::SE2<T>& pose_i,
      const beluga_estimation::Vector3<T>& vel_i,
      view<beluga_estimation::Ref<beluga_estimation::SE2<T>>> pose_j_samples,
      view<beluga_estimation::Ref<beluga_estimation::Vector3<T>>> vel_j_samples,
      RandomNumberGenerator& gen) const {
    const beluga_estimation::SE2<T> expected_pose = pose_i * beluga_estimation::SE2<T>::exp(vel_i * dt_);
    SE2NormalDistribution_<T> pose_dist(expected_pose, Q_pose_);
    pose_dist.Sample(pose_j_samples, gen);

    NormalDistribution_<T, 3> vel_dist(vel_i, Q_vel_);
    vel_dist.Sample(vel_j_samples, gen);
  }

  void CalcMean(
      const beluga_estimation::SE2<T>& pose_i,
      const beluga_estimation::Vector3<T>& vel_i,
      beluga_estimation::Ref<beluga_estimation::SE2<T>> pose_j_out,
      beluga_estimation::Ref<beluga_estimation::Vector3<T>> vel_j_out) const {
    pose_j_out = pose_i * beluga_estimation::SE2<T>::exp(vel_i * dt_);
    vel_j_out = vel_i;
  }

  void CalcMean(
      const beluga_estimation::SE2<T>& pose_i,
      const beluga_estimation::Vector3<T>& vel_i,
      beluga_estimation::Ref<beluga_estimation::SE2<T>> pose_j_out,
      beluga_estimation::Ref<beluga_estimation::Vector3<T>> vel_j_out,
      beluga_estimation::Ref<beluga_estimation::Matrix6<T>> J) const {
    const beluga_estimation::SE2<T> delta = beluga_estimation::SE2<T>::exp(vel_i * dt_);
    pose_j_out = pose_i * delta;
    vel_j_out = vel_i;
    J.setZero();
    // d(pose_j) / d(pose_i) = Ad(delta^{-1}). Ref: sola2018micro, Eq. (65) and (99).
    J.template block<3, 3>(0, 0) = delta.inverse().Adj();
    // d(vel_j) / d(vel_i) = I.
    J.template block<3, 3>(3, 3) = beluga_estimation::Matrix3<T>::Identity();
    // d(pose_j) / d(vel_i) = dt * Jr(vel_i * dt), BCH: Jr(ξ) ≈ I - ad(ξ)/2.
    // Ref: sola2018micro, Eq. (70) (BCH) and App. C (SE2 specifics).
    const beluga_estimation::Vector3<T> w = vel_i * dt_;
    beluga_estimation::Matrix3<T> Jr = beluga_estimation::Matrix3<T>::Identity();
    Jr(0, 1) = w(2) / T{2};
    Jr(1, 0) = -w(2) / T{2};
    Jr(0, 2) = -w(1) / T{2};
    Jr(1, 2) = w(0) / T{2};
    J.template block<3, 3>(0, 3) = dt_ * Jr;
  }

  void CalcCovariance(beluga_estimation::Ref<beluga_estimation::Matrix6<T>> cov) const {
    cov.template block<3, 3>(0, 0) = Q_pose_;
    cov.template block<3, 3>(0, 3).setZero();
    cov.template block<3, 3>(3, 0).setZero();
    cov.template block<3, 3>(3, 3) = Q_vel_;
  }

  void CalcResidual(
      const beluga_estimation::SE2<T>& pose_j,
      const beluga_estimation::Vector3<T>& vel_j,
      const beluga_estimation::SE2<T>& pose_i,
      const beluga_estimation::Vector3<T>& vel_i,
      beluga_estimation::Ref<beluga_estimation::Vector6<T>> r) const {
    const beluga_estimation::SE2<T> expected = pose_i * beluga_estimation::SE2<T>::exp(vel_i * dt_);
    r.template head<3>() = (expected.inverse() * pose_j).log();
    r.template tail<3>() = vel_j - vel_i;
  }

  void CalcResidual(
      const beluga_estimation::SE2<T>& pose_j,
      const beluga_estimation::Vector3<T>& vel_j,
      const beluga_estimation::SE2<T>& pose_i,
      const beluga_estimation::Vector3<T>& vel_i,
      beluga_estimation::Ref<beluga_estimation::Vector6<T>> r,
      beluga_estimation::Ref<beluga_estimation::Matrix6<T>> J) const {
    CalcResidual(pose_j, vel_j, pose_i, vel_i, r);
    J.setZero();
    // First-order BCH approximation of Jr^{-1}(r) ≈ I + ad(r)/2 for pose part.
    // Ref: sola2018micro, Eq. (79) (Log map) and App. C (SE2 specifics).
    const auto r_pose = r.template head<3>();
    auto J_pose = J.template block<3, 3>(0, 0);
    J_pose.setIdentity();
    const T half_theta = r_pose(2) / T{2};
    J_pose(0, 1) = -half_theta;
    J_pose(1, 0) = half_theta;
    J_pose(0, 2) = r_pose(1) / T{2};
    J_pose(1, 2) = -r_pose(0) / T{2};
    // d(vel_j - vel_i) / d(vel_j) = I.
    J.template block<3, 3>(3, 3) = beluga_estimation::Matrix3<T>::Identity();
  }

  T ComputeLogLikelihood(
      view<std_future::span<const T>> dependent_values,
      view<std_future::span<const T>> conditioning_values) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_values.size() == 2 && conditioning_values.size() == 2);
    BELUGA_ESTIMATION_ASSERT(dependent_values[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(dependent_values[1].size() == 3, "velocity variable must have dimension 3");
    BELUGA_ESTIMATION_ASSERT(conditioning_values[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(conditioning_values[1].size() == 3, "velocity variable must have dimension 3");
    const auto pose_j = beluga_estimation::Map<const beluga_estimation::SE2<T>>(dependent_values[0].data());
    const auto vel_j = beluga_estimation::Map<const beluga_estimation::Vector3<T>>(dependent_values[1].data());
    const auto pose_i = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_values[0].data());
    const auto vel_i = beluga_estimation::Map<const beluga_estimation::Vector3<T>>(conditioning_values[1].data());
    return ComputeLogLikelihood(pose_j, vel_j, pose_i, vel_i);
  }

  void TakeSamples(
      view<std_future::span<const T>> conditioning_values,
      view<view<std_future::span<T>>> sampled_dependent_values,
      RandomNumberGenerator& generator) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 2);
    BELUGA_ESTIMATION_ASSERT(conditioning_values[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(conditioning_values[1].size() == 3, "velocity variable must have dimension 3");
    BELUGA_ESTIMATION_ASSERT(sampled_dependent_values.size() == 2);
    const auto pose_i = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_values[0].data());
    const auto vel_i = beluga_estimation::Map<const beluga_estimation::Vector3<T>>(conditioning_values[1].data());
    TakeSamples(
        pose_i, vel_i,
        ranges::views::transform(
            sampled_dependent_values[0],
            [](auto sample) {
              BELUGA_ESTIMATION_ASSERT(sample.size() == 4);
              return beluga_estimation::Map<beluga_estimation::SE2<T>>(sample.data());
            }),
        ranges::views::transform(
            sampled_dependent_values[1],
            [](auto sample) {
              BELUGA_ESTIMATION_ASSERT(sample.size() == 3);
              return beluga_estimation::Map<beluga_estimation::Vector3<T>>(sample.data());
            }),
        generator);
  }

  void CalcMean(
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> mean,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 2);
    BELUGA_ESTIMATION_ASSERT(conditioning_variables[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(conditioning_variables[1].size() == 3, "velocity variable must have dimension 3");
    BELUGA_ESTIMATION_ASSERT(mean.size() == 7);
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");
    const auto pose_i = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_variables[0].data());
    const auto vel_i = beluga_estimation::Map<const beluga_estimation::Vector3<T>>(conditioning_variables[1].data());
    if (jacobian) {
      auto J = beluga_estimation::Map<beluga_estimation::Matrix6<T>>(*jacobian);
      CalcMean(
          pose_i, vel_i, beluga_estimation::Map<beluga_estimation::SE2<T>>(mean.data()),
          beluga_estimation::Map<beluga_estimation::Vector3<T>>(mean.data() + 4), J);
    } else {
      CalcMean(
          pose_i, vel_i, beluga_estimation::Map<beluga_estimation::SE2<T>>(mean.data()),
          beluga_estimation::Map<beluga_estimation::Vector3<T>>(mean.data() + 4));
    }
  }

  void CalcCovariance(
      [[maybe_unused]] view<std_future::span<const T>> conditioning_variables,
      std_future::mdspan<T, 2> covariance) const override {
    auto cov = beluga_estimation::Map<beluga_estimation::Matrix6<T>>(covariance);
    CalcCovariance(cov);
  }

  void CalcResidual(
      view<std_future::span<const T>> dependent_variables,
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> residual,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_variables.size() == 2 && conditioning_variables.size() == 2);
    BELUGA_ESTIMATION_ASSERT(dependent_variables[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(dependent_variables[1].size() == 3, "velocity variable must have dimension 3");
    BELUGA_ESTIMATION_ASSERT(conditioning_variables[0].size() == 4);
    BELUGA_ESTIMATION_DEMAND(conditioning_variables[1].size() == 3, "velocity variable must have dimension 3");
    BELUGA_ESTIMATION_ASSERT(residual.size() == 6);
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");
    const auto pose_j = beluga_estimation::Map<const beluga_estimation::SE2<T>>(dependent_variables[0].data());
    const auto vel_j = beluga_estimation::Map<const beluga_estimation::Vector3<T>>(dependent_variables[1].data());
    const auto pose_i = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_variables[0].data());
    const auto vel_i = beluga_estimation::Map<const beluga_estimation::Vector3<T>>(conditioning_variables[1].data());
    auto r = beluga_estimation::Map<beluga_estimation::Vector6<T>>(residual.data());
    if (jacobian) {
      auto J = beluga_estimation::Map<beluga_estimation::Matrix6<T>>(*jacobian);
      CalcResidual(pose_j, vel_j, pose_i, vel_i, r, J);
    } else {
      CalcResidual(pose_j, vel_j, pose_i, vel_i, r);
    }
  }

 private:
  const Identifier& PoseIId() const { return conditioning_ids_[0]; }
  const Identifier& PoseJId() const { return dependent_ids_[0]; }

  static T CalculateDt(const Identifier& pose_i_id, const Identifier& pose_j_id) {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    return duration_cast<duration<T>>(pose_j_id.timestamp - pose_i_id.timestamp).count();
  }

  std::array<Identifier, 2> conditioning_ids_;  // [pose_i, vel_i]
  std::array<Identifier, 2> dependent_ids_;     // [pose_j, vel_j]
  T dt_;
  beluga_estimation::Matrix3<T> Q_pose_;
  beluga_estimation::Matrix3<T> Q_vel_;
};

using SE2ConstantVelocityKinematicConstraint = SE2ConstantVelocityKinematicConstraint_<DefaultScalar>;

}  // namespace beluga_estimation

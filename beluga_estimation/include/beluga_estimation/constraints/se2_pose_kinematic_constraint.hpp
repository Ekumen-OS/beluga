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
#include <beluga_estimation/distributions/se2_normal_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief Pose-only kinematic constraint p(x_j | x_i) = N(x_i * delta, Q).
 *
 * When delta is identity (default), models zero relative displacement between poses.
 *
 * @tparam T Scalar type (float or double).
 */
template <typename T>
class SE2PoseKinematicConstraint_ : public LocallyNormalConstraint_<T> {
 public:
  /// @brief Construct with identity delta (zero-displacement assumption).
  SE2PoseKinematicConstraint_(
      Identifier id,
      Identifier pose_i_id,
      Identifier pose_j_id,
      beluga_estimation::Matrix3<T> process_noise)
      : SE2PoseKinematicConstraint_(
            std::move(id),
            std::move(pose_i_id),
            std::move(pose_j_id),
            std::move(process_noise),
            beluga_estimation::SE2<T>{}) {}

  /// @brief Construct with an explicit relative transform; models p(x_j | x_i) = N(x_i * delta, Q).
  SE2PoseKinematicConstraint_(
      Identifier id,
      Identifier pose_i_id,
      Identifier pose_j_id,
      beluga_estimation::Matrix3<T> process_noise,
      beluga_estimation::SE2<T> delta)
      : LocallyNormalConstraint_<T>(std::move(id)),
        pose_i_id_(std::move(pose_i_id)),
        pose_j_id_(std::move(pose_j_id)),
        delta_(std::move(delta)),
        Q_(std::move(process_noise)) {
    BELUGA_ESTIMATION_DEMAND(IsCovarianceMatrix(Q_), "process_noise must be symmetric positive definite");
  }

  bool HasDependentVariables() const override { return true; }
  bool HasConditioningVariables() const override { return true; }

  std_future::span<const Identifier> DependentVariables() const override { return {&pose_j_id_, 1}; }

  std_future::span<const Identifier> ConditioningVariables() const override { return {&pose_i_id_, 1}; }

  [[nodiscard]] std::size_t ResidualSize() const override { return beluga_estimation::SE2<T>::DoF; }

  T ComputeLogLikelihood(const beluga_estimation::SE2<T>& pose_j, const beluga_estimation::SE2<T>& pose_i) const {
    SE2NormalDistribution_<T> dist(pose_i * delta_, Q_);
    return dist.LogProbability(std_future::span<const T>{pose_j.data(), 4});
  }

  void TakeSamples(
      const beluga_estimation::SE2<T>& pose_i,
      view<beluga_estimation::Ref<beluga_estimation::SE2<T>>> pose_j_samples,
      RandomNumberGenerator& gen) const {
    SE2NormalDistribution_<T> dist(pose_i * delta_, Q_);
    dist.Sample(pose_j_samples, gen);
  }

  void CalcMean(const beluga_estimation::SE2<T>& pose_i, beluga_estimation::Ref<beluga_estimation::SE2<T>> mean_out)
      const {
    mean_out = pose_i * delta_;
  }

  void CalcMean(
      const beluga_estimation::SE2<T>& pose_i,
      beluga_estimation::Ref<beluga_estimation::SE2<T>> mean_out,
      beluga_estimation::Ref<beluga_estimation::Matrix3<T>> J) const {
    mean_out = pose_i * delta_;
    J = delta_.inverse().Adj();
  }

  void CalcCovariance(beluga_estimation::Ref<beluga_estimation::Matrix3<T>> cov) const { cov = Q_; }

  void CalcResidual(
      const beluga_estimation::SE2<T>& pose_j,
      const beluga_estimation::SE2<T>& pose_i,
      beluga_estimation::Ref<beluga_estimation::Vector3<T>> r) const {
    r = ((pose_i * delta_).inverse() * pose_j).log();
  }

  void CalcResidual(
      const beluga_estimation::SE2<T>& pose_j,
      const beluga_estimation::SE2<T>& pose_i,
      beluga_estimation::Ref<beluga_estimation::Vector3<T>> r,
      beluga_estimation::Ref<beluga_estimation::Matrix3<T>> J) const {
    CalcResidual(pose_j, pose_i, r);
    // First-order BCH approximation of Jr^{-1}(r) ≈ I + ad(r)/2.
    // Ref: sola2018micro, Eq. (79) (Log map) and App. C (SE2 specifics).
    J.setIdentity();
    const T half_theta = r(2) / T{2};
    J(0, 1) = -half_theta;
    J(1, 0) = half_theta;
    J(0, 2) = r(1) / T{2};
    J(1, 2) = -r(0) / T{2};
  }

  T ComputeLogLikelihood(
      view<std_future::span<const T>> dependent_values,
      view<std_future::span<const T>> conditioning_values) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_values.size() == 1 && conditioning_values.size() == 1);
    BELUGA_ESTIMATION_ASSERT(dependent_values[0].size() == 4 && conditioning_values[0].size() == 4);
    const auto pose_j = beluga_estimation::Map<const beluga_estimation::SE2<T>>(dependent_values[0].data());
    const auto pose_i = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_values[0].data());
    return ComputeLogLikelihood(pose_j, pose_i);
  }

  void TakeSamples(
      view<std_future::span<const T>> conditioning_values,
      view<view<std_future::span<T>>> sampled_dependent_values,
      RandomNumberGenerator& generator) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_values.size() == 1 && conditioning_values[0].size() == 4);
    BELUGA_ESTIMATION_ASSERT(sampled_dependent_values.size() == 1);
    const auto pose_i = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_values[0].data());
    TakeSamples(
        pose_i,
        ranges::views::transform(
            sampled_dependent_values[0],
            [](auto sample) {
              BELUGA_ESTIMATION_ASSERT(sample.size() == 4);
              return beluga_estimation::Map<beluga_estimation::SE2<T>>(sample.data());
            }),
        generator);
  }

  void CalcMean(
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> mean,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(conditioning_variables.size() == 1 && conditioning_variables[0].size() == 4);
    BELUGA_ESTIMATION_ASSERT(mean.size() == 4);
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");
    const auto pose_i = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_variables[0].data());
    if (jacobian) {
      auto J = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(*jacobian);
      CalcMean(pose_i, beluga_estimation::Map<beluga_estimation::SE2<T>>(mean.data()), J);
    } else {
      CalcMean(pose_i, beluga_estimation::Map<beluga_estimation::SE2<T>>(mean.data()));
    }
  }

  void CalcCovariance(
      [[maybe_unused]] view<std_future::span<const T>> conditioning_variables,
      std_future::mdspan<T, 2> covariance) const override {
    auto cov = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(covariance);
    CalcCovariance(cov);
  }

  void CalcResidual(
      view<std_future::span<const T>> dependent_variables,
      view<std_future::span<const T>> conditioning_variables,
      std_future::span<T> residual,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(dependent_variables.size() == 1 && conditioning_variables.size() == 1);
    BELUGA_ESTIMATION_ASSERT(dependent_variables[0].size() == 4 && conditioning_variables[0].size() == 4);
    BELUGA_ESTIMATION_ASSERT(residual.size() == 3);
    BELUGA_ESTIMATION_DEMAND(!hessian, "hessian computation is not supported");
    const auto pose_j = beluga_estimation::Map<const beluga_estimation::SE2<T>>(dependent_variables[0].data());
    const auto pose_i = beluga_estimation::Map<const beluga_estimation::SE2<T>>(conditioning_variables[0].data());
    auto r = beluga_estimation::Map<beluga_estimation::Vector3<T>>(residual.data());
    if (jacobian) {
      auto J = beluga_estimation::Map<beluga_estimation::Matrix3<T>>(*jacobian);
      CalcResidual(pose_j, pose_i, r, J);
    } else {
      CalcResidual(pose_j, pose_i, r);
    }
  }

 private:
  Identifier pose_i_id_;
  Identifier pose_j_id_;
  beluga_estimation::SE2<T> delta_;
  beluga_estimation::Matrix3<T> Q_;
};

using SE2PoseKinematicConstraint = SE2PoseKinematicConstraint_<DefaultScalar>;

}  // namespace beluga_estimation

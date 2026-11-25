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

#include <Eigen/Eigenvalues>

#include "planar_projection.hpp"

namespace beluga_visual_odometry {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFiniteDifferenceStep = 1.0e-6;

Eigen::Vector3d PlanarVector(
    const Sophus::SE3d& camera_pose,
    const Sophus::SE3d& base_from_camera) {
  const Sophus::SE3d pose =
      base_from_camera * camera_pose * base_from_camera.inverse();
  const Eigen::Matrix3d rotation = pose.so3().matrix();
  return {
      pose.translation().x(),
      pose.translation().z(),
      std::atan2(rotation(0, 2), rotation(0, 0))};
}

bool MakePositiveSemidefinite(Eigen::Matrix3d& covariance) {
  if (!covariance.allFinite()) {
    return false;
  }
  covariance = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -1.0e-8) {
    return false;
  }
  covariance = solver.eigenvectors() *
      solver.eigenvalues().cwiseMax(1.0e-12).asDiagonal() *
      solver.eigenvectors().transpose();
  return covariance.allFinite();
}

}  // namespace

Eigen::Vector3d PlanarProjector::Project(
    const Sophus::SE3d& relative_pose) const {
  return PlanarVector(relative_pose, config_.T_base_camera);
}

bool PlanarProjector::ProjectCovariance(
    const Sophus::SE3d& relative_pose,
    const Eigen::Matrix<double, 6, 6>& covariance,
    Eigen::Matrix3d& projected_covariance) const {
  Eigen::Matrix<double, 3, 6> jacobian;
  const Eigen::Vector3d nominal = Project(relative_pose);
  for (int column = 0; column < 6; ++column) {
    Eigen::Matrix<double, 6, 1> perturbation =
        Eigen::Matrix<double, 6, 1>::Zero();
    perturbation(column) = kFiniteDifferenceStep;
    const Eigen::Vector3d perturbed = Project(
        Sophus::SE3d::exp(perturbation) * relative_pose);
    jacobian.col(column) = (perturbed - nominal) / kFiniteDifferenceStep;
    jacobian(2, column) = std::remainder(
        perturbed.z() - nominal.z(), 2.0 * kPi) / kFiniteDifferenceStep;
  }
  projected_covariance = jacobian * covariance * jacobian.transpose();
  return MakePositiveSemidefinite(projected_covariance);
}

}  // namespace beluga_visual_odometry

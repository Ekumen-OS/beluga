// Copyright 2025 Ekumen, Inc.
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

#include <beluga_ros/ndt_ellipsoid.hpp>

#include <Eigen/Core>
#include <beluga/eigen_compatibility.hpp>

namespace beluga_ros {

namespace {

template <typename RealScalar>
inline bool is_approx(RealScalar x, RealScalar y, RealScalar prec = Eigen::NumTraits<RealScalar>::dummy_precision()) {
  return std::abs(x - y) < prec;
}

}  // namespace

namespace detail {

bool use_mean_covariance(
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>>& eigen_solver,
    const beluga::NDTCell<3> cell,
    beluga_ros::msg::Marker& marker) {
  eigen_solver.compute(cell.covariance);
  if (eigen_solver.info() != Eigen::Success) {
    return false;
  }

  // Compute the rotation based on the covariance matrix
  const auto eigenvectors = eigen_solver.eigenvectors().real();
  const auto eigenvalues = eigen_solver.eigenvalues();

  Eigen::Matrix3d rotation_matrix;
  Eigen::Vector3d scalevector;
  rotation_matrix << eigenvectors.col(0), eigenvectors.col(1), eigenvectors.col(2);
  scalevector = Eigen::Vector3d{eigenvalues.x(), eigenvalues.y(), eigenvalues.z()};

  if (!is_approx(std::abs(rotation_matrix.determinant()), 1.0)) {
    return false;
  }

  // Permute columns to ensure the rotation is in fact a rotation
  if (rotation_matrix.determinant() < 0) {
    rotation_matrix << eigenvectors.col(1), eigenvectors.col(0), eigenvectors.col(2);
    scalevector = Eigen::Vector3d{eigenvalues.y(), eigenvalues.x(), eigenvalues.z()};
  }

  const auto rotation = Eigen::Quaterniond{rotation_matrix};

  // Fill in the message
  marker.type = beluga_ros::msg::Marker::SPHERE;
  marker.action = beluga_ros::msg::Marker::ADD;

  marker.pose.position.x = cell.mean[0];
  marker.pose.position.y = cell.mean[1];
  marker.pose.position.z = cell.mean[2];

  marker.pose.orientation.x = rotation.x();
  marker.pose.orientation.y = rotation.y();
  marker.pose.orientation.z = rotation.z();
  marker.pose.orientation.w = rotation.w();

  constexpr float k_scale_factor = 4.0F;
  marker.scale.x = std::sqrt(scalevector.x()) * k_scale_factor;
  marker.scale.y = std::sqrt(scalevector.y()) * k_scale_factor;
  marker.scale.z = std::sqrt(scalevector.z()) * k_scale_factor;

  marker.color.r = 0.0F;
  marker.color.g = 1.0F;
  marker.color.b = 0.0F;
  marker.color.a = 1.0F;

  return true;
}

void use_cell_size(const Eigen::Vector<int, 3>& position, double size, beluga_ros::msg::Marker& marker) {
  marker.type = beluga_ros::msg::Marker::CUBE;
  marker.action = beluga_ros::msg::Marker::ADD;

  marker.pose.position.x = position[0];
  marker.pose.position.y = position[1];
  marker.pose.position.z = position[2];

  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;

  marker.color.r = 1.0F;
  marker.color.g = 0.0F;
  marker.color.b = 0.0F;
  marker.color.a = 1.0F;
}

}  // namespace detail

}  // namespace beluga_ros

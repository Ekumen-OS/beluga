// Copyright 2024 Ekumen, Inc.
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

#ifndef BELUGA_ROS_NDT_ELLIPSOID_HPP
#define BELUGA_ROS_NDT_ELLIPSOID_HPP

#include <beluga_ros/messages.hpp>

/**
 * \file
 * \brief Utilities for NDT I/O over ROS interfaces.
 */

namespace beluga_ros {

namespace {

template <typename RealScalar>
inline bool isApprox(RealScalar x, RealScalar y, RealScalar prec = Eigen::NumTraits<RealScalar>::dummy_precision()) {
  return std::abs(x - y) < prec;
}

}  // namespace

inline bool use_mean_covariance(auto& eigenSolver, const auto& cell, auto& marker) {
  eigenSolver.compute(cell.covariance);
  if (eigenSolver.info() != Eigen::Success) {
    return false;
  }

  // Compute the rotation based on the covariance matrix
  const auto eigenvectors = eigenSolver.eigenvectors().real();
  const auto eigenvalues = eigenSolver.eigenvalues();

  Eigen::Matrix3d rotationMatrix;
  Eigen::Vector3d scalevector;
  rotationMatrix << eigenvectors.col(0), eigenvectors.col(1), eigenvectors.col(2);
  scalevector = Eigen::Vector3d{eigenvalues.x(), eigenvalues.y(), eigenvalues.z()};

  // Permutation
  if (!isApprox(rotationMatrix.determinant(), 1.0)) {
    rotationMatrix << eigenvectors.col(1), eigenvectors.col(0), eigenvectors.col(2);
    scalevector = Eigen::Vector3d{eigenvalues.y(), eigenvalues.x(), eigenvalues.z()};
  }

  if (!isApprox(rotationMatrix.determinant(), 1.0)) {
    return false;
  }
  const auto rotation = Eigen::Quaterniond{rotationMatrix};

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

  constexpr float kScaleFactor = 4.0f;
  marker.scale.x = std::sqrt(scalevector.x()) * kScaleFactor;
  marker.scale.y = std::sqrt(scalevector.y()) * kScaleFactor;
  marker.scale.z = std::sqrt(scalevector.z()) * kScaleFactor;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  return true;
}

inline void use_cell_size(const auto& position, const auto& size, auto& marker) {
  marker.type = beluga_ros::msg::Marker::CUBE;
  marker.action = beluga_ros::msg::Marker::ADD;

  marker.pose.position.x = position[0];
  marker.pose.position.y = position[1];
  marker.pose.position.z = position[2];

  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
}

/// Assign an ellipsoid to each cell of a SparseValueGrid. A cube is used instead if the distribution of the
/// cell is not suitable for the rotation matrix creation.
/**
 * \param grid A SparseValueGrid that contains cells representing obstacles.
 * \return A message with the ellipsoids or cubes.
 * \param[out] message Markers message to be assigned.
 * \param[out] cubesGenerated Is set to true if there were problems with covariance matrices from cells.
 * \tparam MapType Container that maps from Eigen::Vector<int, NDim> to the type of the cell. See [SparseValueGrid]
 * (https://ekumen-os.github.io/beluga/packages/beluga/docs/_doxygen/generated/reference/html/classbeluga_1_1SparseValueGrid.html).
 * \tparam NDim Dimension of the grid.
 */
template <typename MapType, int NDim>
beluga_ros::msg::MarkerArray assign_obstacle_map(
    const beluga::SparseValueGrid<MapType, NDim>& grid,
    beluga_ros::msg::MarkerArray& message,
    bool& cubesGenerated) {
  cubesGenerated = false;
  // Get data from the grid
  auto& map = grid.data();

  // Clean up the message
  beluga_ros::msg::Marker marker;
  marker.ns = "obstacles";
  marker.action = beluga_ros::msg::Marker::DELETEALL;
  message.markers.push_back(marker);

  // Add the markers
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> eigenSolver;
  for (auto [index, entry] : ranges::views::enumerate(map)) {
    const auto& [cellCenter, cell] = entry;
    beluga_ros::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.id = index;
    marker.ns = "obstacles";

    // Try to create an ellipsoid with values of the cell
    if (!use_mean_covariance(eigenSolver, cell, marker)) {
      // Create a cube based on the resolution of the grid
      cubesGenerated = true;
      use_cell_size(cellCenter, grid.resolution(), marker);
    }

    // Add the marker
    message.markers.push_back(marker);
  }

  return message;
}

}  // namespace beluga_ros

#endif  // BELUGA_ROS_NDT_ELLIPSOID_HPP

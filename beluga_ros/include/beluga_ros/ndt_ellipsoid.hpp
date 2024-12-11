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

inline void use_mean_covariance(const auto& mean, const auto& eigenSolver, auto& marker){
  marker.type = beluga_ros::msg::Marker::SPHERE;
  marker.action = beluga_ros::msg::Marker::ADD;
  
  // Compute the rotation based on the covariance matrix
  const auto eigenvectors = eigenSolver.eigenvectors().real();
  const auto eigenvalues = eigenSolver.eigenvalues();

  Eigen::Matrix3d rotationMatrix;
  Eigen::Vector3d scalevector;
  rotationMatrix << eigenvectors.col(0), eigenvectors.col(1), eigenvectors.col(2);
  scalevector = Eigen::Vector3d{eigenvalues.x(), eigenvalues.y(), eigenvalues.z()};

  // Permutation
  if (std::abs(rotationMatrix.determinant() - 1.0) > Eigen::NumTraits<double>::dummy_precision()) {
    rotationMatrix << eigenvectors.col(1), eigenvectors.col(0), eigenvectors.col(2);
    scalevector = Eigen::Vector3d{eigenvalues.y(), eigenvalues.x(), eigenvalues.z()};
  }

  Eigen::Quaterniond rotation{1.0f, 0.0f, 0.0f, 0.0f};
  if (std::abs(rotationMatrix.determinant() - 1.0) < Eigen::NumTraits<double>::dummy_precision()) {
    rotation = Eigen::Quaterniond{rotationMatrix};
  }

  marker.pose.position.x = mean[0];
  marker.pose.position.y = mean[1];
  marker.pose.position.z = mean[2];

  marker.pose.orientation.x = rotation.x();
  marker.pose.orientation.y = rotation.y();
  marker.pose.orientation.z = rotation.z();
  marker.pose.orientation.w = rotation.w();

  float scaleConstant = 5.0f;
  marker.scale.x = scalevector.x() * scaleConstant;
  marker.scale.y = scalevector.y() * scaleConstant;
  marker.scale.z = scalevector.z() * scaleConstant;
}

inline void use_cell_size(const auto& position, const auto& size, auto& marker){
  marker.type = beluga_ros::msg::Marker::CUBE;
  marker.action = beluga_ros::msg::Marker::ADD;

  marker.pose.position.x = position[0];
  marker.pose.position.y = position[1];
  marker.pose.position.z = position[2];

  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
}

/// Assign an ellipsoid to each cell of a SparseValueGrid. A cube is used instead if the distribution of the
/// cell is not suitable for the rotation matrix creation.
 /** 
  * \param grid A SparseValueGrid that contains cells representing obstacles.
  * \return A message with the ellipsoids or cubes.
  * \tparam MapType Container that maps from Eigen::Vector<int, NDim> to the type of the cell. See [SparseValueGrid]
  * (https://ekumen-os.github.io/beluga/packages/beluga/docs/_doxygen/generated/reference/html/classbeluga_1_1SparseValueGrid.html).
  * \tparam NDim Dimension of the grid.
  */ 
template <typename MapType, int NDim>
beluga_ros::msg::MarkerArray assign_obstacle_map(const beluga::SparseValueGrid<MapType, NDim>& grid) {
  // Get data from the grid
  auto& map = grid.data();

  // Clean up the message
  beluga_ros::msg::MarkerArray message{};
  {
    beluga_ros::msg::Marker marker;
    marker.ns = "obstacles";
    marker.action = beluga_ros::msg::Marker::DELETEALL;
    message.markers.push_back(marker);
  }

  // Add the markers
  for (auto [index, entry] : ranges::views::enumerate(map)) {
    const auto [cellCenter, cell] = entry;
    beluga_ros::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.id = index;
    marker.ns = "obstacles";

    const auto eigenSolver = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>>{cell.covariance};
    if(eigenSolver.info() == Eigen::Success){
      // Create an ellipsoid with values of the cell
      use_mean_covariance(cell.mean, eigenSolver, marker);
    }else{
      // Create a cube based on the resolution of the grid
      use_cell_size(cellCenter, grid.resolution(), marker);
    }

    float color[] = {0.0f, 1.0f, 0.0f, 1.0f};
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    // Add the marker
    message.markers.push_back(marker);
  }

  return message;
}

}  // namespace beluga_ros

#endif  // BELUGA_ROS_NDT_ELLIPSOID_HPP

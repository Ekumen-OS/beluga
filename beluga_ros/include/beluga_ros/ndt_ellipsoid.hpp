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

#include <Eigen/Core>
#include <beluga/eigen_compatibility.hpp>

#include <range/v3/view/enumerate.hpp>

#include <beluga/sensor/data/sparse_value_grid.hpp>
#include <beluga/sensor/data/ndt_cell.hpp>

/**
 * \file
 * \brief Utilities for NDT I/O over ROS interfaces.
 */

namespace beluga_ros {

bool use_mean_covariance(
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>>& eigenSolver,
    const beluga::NDTCell<3> cell,
    beluga_ros::msg::Marker& marker);

void use_cell_size(
    const Eigen::Vector<int, 3>& position,
    double size,
    beluga_ros::msg::Marker& marker);

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
    marker.id = index+1;
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

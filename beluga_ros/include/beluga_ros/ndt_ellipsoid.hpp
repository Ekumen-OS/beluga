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

#ifndef BELUGA_ROS_NDT_ELLIPSOID_HPP
#define BELUGA_ROS_NDT_ELLIPSOID_HPP

#include <Eigen/Core>
#include <beluga/eigen_compatibility.hpp>

#include <range/v3/view/enumerate.hpp>

#include <beluga/sensor/data/ndt_cell.hpp>
#include <beluga/sensor/data/sparse_value_grid.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/**
 * \file
 * \brief Utilities for NDT I/O over ROS interfaces.
 */

namespace beluga_ros {

namespace detail {

/// Create an ellipoid based on NDT cell data (eigenvalues and eigenvectors) contained in the received marker.
/**
 * \param eigen_solver Precreated eigen solver of the NDT cell.
 * \param cell NDT cell.
 * \param marker Marker that will contain the message.
 * \return A boolean set to false if the ellipsoid could not be created (because the covariances are
 * non-diagonalizable).
 */
bool use_mean_covariance(
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>>& eigen_solver,
    beluga::NDTCell<3> cell,
    visualization_msgs::msg::Marker& marker);

/// Create a cube contained in the received marker.
/**
 * \param position Center of the cube.
 * \param size Size of the edges.
 * \param marker Marker that will contain the message.
 */
void use_cell_size(const Eigen::Vector<int, 3>& position, double size, visualization_msgs::msg::Marker& marker);

}  // namespace detail

/// Assign an ellipsoid to each cell of a SparseValueGrid. A cube is used instead if the distribution of the
/// cell is not suitable for the rotation matrix creation.
/**
 * \param grid A SparseValueGrid that contains cells representing obstacles.
 * \param[out] message A MarkerArray that will contain the shapes
 * \return A std::pair with the MarkerArray containing the shapes and a boolean indicating if at least one cube was
 * generated. \tparam MapType Container that maps from Eigen::Vector<int, NDim> to the type of the cell. See
 * [SparseValueGrid]
 * (https://ekumen-os.github.io/beluga/packages/beluga/docs/_doxygen/generated/reference/html/classbeluga_1_1SparseValueGrid.html).
 * \tparam NDim Dimension of the grid.
 */
template <typename MapType, int NDim>
std::pair<visualization_msgs::msg::MarkerArray, bool> assign_obstacle_map(
    const beluga::SparseValueGrid<MapType, NDim>& grid,
    visualization_msgs::msg::MarkerArray& message) {
  bool cubes_generated = false;
  // Get data from the grid
  auto& map = grid.data();

  // Clean up the message
  visualization_msgs::msg::Marker marker;
  marker.ns = "obstacles";
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  message.markers.push_back(marker);

  // Add the markers
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> eigen_solver;
  for (auto [index, entry] : ranges::views::enumerate(map)) {
    const auto& [cell_center, cell] = entry;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.id = index + 1;
    marker.ns = "obstacles";

    // Try to create an ellipsoid with values of the cell
    if (!beluga_ros::detail::use_mean_covariance(eigen_solver, cell, marker)) {
      // Create a cube based on the resolution of the grid
      cubes_generated = true;
      beluga_ros::detail::use_cell_size(cell_center, grid.resolution(), marker);
    }

    // Add the marker
    message.markers.push_back(marker);
  }

  return {message, cubes_generated};
}

}  // namespace beluga_ros

#endif  // BELUGA_ROS_NDT_ELLIPSOID_HPP

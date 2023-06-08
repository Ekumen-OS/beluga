// Copyright 2022-2023 Ekumen, Inc.
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

#ifndef BELUGA_ALGORITHM_DISTANCE_MAP_HPP
#define BELUGA_ALGORITHM_DISTANCE_MAP_HPP

#include <queue>
#include <vector>

#include <range/v3/range/access.hpp>
#include <range/v3/range/primitives.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/enumerate.hpp>

#include <Eigen/Core>

/**
 * \file
 * \brief Implementation of algorithm to calculate distance from obstacles.
 */

/// Returns a map where the value of each cell is the distance to the nearest obstacle.
/**
 * The algorithm uses O(N) time and memory, where `N=ranges::size(obstacle_map)`.
 *
 * \tparam Range A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range).
 *  Its value type must be bool.
 * \tparam DistanceFunction A callable type, its prototype must be
 *  (std::size_t, std::size_t) -> DistanceType. DistanceType must be an scalar type.
 * \tparam NeighborsFunction A callable type, its prototype must be
 *  (std::size_t) -> NeighborsT, where NeighborsT is a
 *    [Range](https://en.cppreference.com/w/cpp/ranges/range)
 *    with value type std::size_t.
 * \param width The width of the obstacle map.
 * \param height The height of the obstacle map.
 * \param obstacle_map A map that represents obstacles in an environment.
 *  If the value of a cell is True, the cell has an obstacle.
 * \param distance_function Given the indexes of two cells in the map i and j,
 *  obstacle_map(i, j) must return the distance between the two cells.
 * \param neighbors_function Given the index i of one cell in the map,
 *  neighbors_function(i) returns the cell indexes of neighbor cells in the
 *  obstacle map.
 * \return A map where each cell value is the distance to the nearest object.
 */
template <class Range, class DistanceFunction, class NeighborsFunction>
auto nearest_obstacle_distance_map(
    const std::size_t width,
    const std::size_t height,
    Range&& obstacle_map,
    DistanceFunction&& distance_function,
    NeighborsFunction&& neighbors_function) {
  struct IndexPair {
    Eigen::Vector2i nearest_obstacle_cell;
    Eigen::Vector2i cell;
  };

  using DistanceType = std::invoke_result_t<DistanceFunction, Eigen::Vector2i, Eigen::Vector2i>;

  const auto distance_map_size = width * height;
  auto distance_map = std::vector<DistanceType>(distance_map_size);
  auto visited = std::vector<bool>(distance_map_size, false);

  const auto linear_index = [width](const auto& cell) { return cell.y() * width + cell.x(); };

  auto compare = [&distance_map, linear_index](const IndexPair& first, const IndexPair& second) {
    return distance_map[linear_index(first.cell)] > distance_map[linear_index(second.cell)];
  };

  auto queue = std::priority_queue<IndexPair, std::vector<IndexPair>, decltype(compare)>{compare};

  for (const auto& cell : obstacle_map | ranges::views::common) {
    const auto index = linear_index(cell);
    visited[index] = true;
    distance_map[index] = 0;
    queue.push(IndexPair{cell, cell});  // The nearest obstacle is itself
  }

  while (!queue.empty()) {
    auto parent = queue.top();
    queue.pop();
    for (const auto& cell : neighbors_function(parent.cell)) {
      const auto index = linear_index(cell);
      if (!visited[index]) {
        visited[index] = true;
        distance_map[index] = distance_function(parent.nearest_obstacle_cell, cell);
        queue.push(IndexPair{parent.nearest_obstacle_cell, cell});
      }
    }
  }

  return distance_map;
}

#endif

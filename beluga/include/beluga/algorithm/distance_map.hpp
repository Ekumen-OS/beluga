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
#include <range/v3/view/enumerate.hpp>

/**
 * \file
 * \brief Implementation of algorithm to calculate distance from obstacles.
 */

namespace beluga {

/// Returns a map where the value of each cell is the distance to the nearest obstacle.
/**
 * The algorithm uses O(N) time and memory, where `N=ranges::size(obstacle_map)`.
 *
 * \tparam Range A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range).
 *  Its value type must be bool.
 * \tparam DistanceFunction A callable type, its prototype must be
 *  (std::size_t, std::size_t) -> DistanceType. DistanceType must be an scalar type.
 * \tparam NeighborsFunction A callabe type, its prototype must be
 *  (std::size_t) -> NeighborsT, where NeighborsT is a
 *    [Range](https://en.cppreference.com/w/cpp/ranges/range)
 *    with value type std::size_t.
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
    Range&& obstacle_map,
    DistanceFunction&& distance_function,
    NeighborsFunction&& neighbors_function) {
  struct IndexPair {
    std::size_t nearest_obstacle_index;
    std::size_t index;
  };

  using DistanceType = std::invoke_result_t<DistanceFunction, std::size_t, std::size_t>;
  auto distance_map = std::vector<DistanceType>(ranges::size(obstacle_map));
  auto visited = std::vector<bool>(ranges::size(obstacle_map), false);

  auto compare = [&distance_map](const IndexPair& first, const IndexPair& second) {
    return distance_map[first.index] > distance_map[second.index];
  };
  auto queue = std::priority_queue<IndexPair, std::vector<IndexPair>, decltype(compare)>{compare};

  for (auto [index, is_obstacle] : ranges::views::enumerate(obstacle_map)) {
    if (is_obstacle) {
      visited[index] = true;
      distance_map[index] = 0;
      queue.push(IndexPair{index, index});  // The nearest obstacle is itself
    }
  }

  while (!queue.empty()) {
    auto parent = queue.top();
    queue.pop();
    for (const std::size_t index : neighbors_function(parent.index)) {
      if (!visited[index]) {
        visited[index] = true;
        distance_map[index] = distance_function(parent.nearest_obstacle_index, index);
        queue.push(IndexPair{parent.nearest_obstacle_index, index});
      }
    }
  }

  return distance_map;
}

}  // namespace beluga

#endif

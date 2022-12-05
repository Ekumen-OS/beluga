// Copyright 2022 Ekumen, Inc.
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

#include <queue>
#include <vector>

template <class Range, class DistanceFunction, class NeighborsFunction>
auto nearest_obstacle_distance_map(
    Range&& map,
    DistanceFunction&& distance_function,
    NeighborsFunction&& neighbors_function) {
  struct IndexPair {
    std::size_t nearest_obstacle_index;
    std::size_t index;
  };

  using DistanceType = std::invoke_result_t<DistanceFunction, std::size_t, std::size_t>;
  auto distance_map = std::vector<DistanceType>(map.size());
  auto visited = std::vector<bool>(map.size(), false);

  auto compare = [&distance_map](const IndexPair& first, const IndexPair& second) {
    return distance_map[first.index] > distance_map[second.index];
  };
  auto queue = std::priority_queue<IndexPair, std::vector<IndexPair>, decltype(compare)>{compare};

  auto begin = map.begin();
  for (std::size_t index = 0; index < map.size(); ++index) {
    bool is_obstacle = *(begin + index);
    if (is_obstacle) {
      visited[index] = true;
      distance_map[index] = 0;
      queue.push(IndexPair{index, index});  // The nearest obstacle is itself
    }
  }

  while (!queue.empty()) {
    auto parent = queue.top();
    queue.pop();
    for (std::size_t index : neighbors_function(parent.index)) {
      if (!visited[index]) {
        visited[index] = true;
        distance_map[index] = distance_function(parent.nearest_obstacle_index, index);
        queue.push(IndexPair{parent.nearest_obstacle_index, index});
      }
    }
  }

  return distance_map;
}

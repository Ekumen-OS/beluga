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

#include <vector>

#include <gmock/gmock.h>

#include <beluga/algorithm/distance_map.hpp>

#include <Eigen/Core>

namespace {

using CellType = Eigen::Vector2i;

auto array_distance(const CellType& first, const CellType& second) {
  return (first - second).norm();
}

auto make_neighbors_function(std::size_t size) {
  return [=](const CellType& cell) {
    auto result = std::vector<CellType>{};
    if (cell.x() > 0) {
      result.emplace_back(cell.x() - 1, cell.y());
    }
    if (static_cast<std::size_t>(cell.x()) < size - 1) {
      result.emplace_back(cell.x() + 1, cell.y());
    }
    return result;
  };
}

TEST(DistanceMap, Empty) {
  auto map = std::vector<CellType>{};
  auto distance_map = nearest_obstacle_distance_map(6, 1, map, array_distance, make_neighbors_function(6));
  ASSERT_THAT(distance_map, testing::ElementsAre(0, 0, 0, 0, 0, 0));
}

TEST(DistanceMap, Full) {
  auto map = std::vector<CellType>{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}};
  auto distance_map = nearest_obstacle_distance_map(6, 1, map, array_distance, make_neighbors_function(6));
  ASSERT_THAT(distance_map, testing::ElementsAre(0, 0, 0, 0, 0, 0));
}

TEST(DistanceMap, Case1) {
  auto map = std::vector<CellType>{{1, 0}, {5, 0}};
  auto distance_map = nearest_obstacle_distance_map(6, 1, map, array_distance, make_neighbors_function(6));
  ASSERT_THAT(distance_map, testing::ElementsAre(1, 0, 1, 2, 1, 0));
}

TEST(DistanceMap, Case2) {
  auto map = std::vector<CellType>{{0, 0}, {1, 0}};
  auto distance_map = nearest_obstacle_distance_map(6, 1, map, array_distance, make_neighbors_function(6));
  ASSERT_THAT(distance_map, testing::ElementsAre(0, 0, 1, 2, 3, 4));
}

TEST(DistanceMap, Case3) {
  auto map = std::vector<CellType>{{5, 0}};
  auto distance_map = nearest_obstacle_distance_map(6, 1, map, array_distance, make_neighbors_function(6));
  ASSERT_THAT(distance_map, testing::ElementsAre(5, 4, 3, 2, 1, 0));
}

}  // namespace

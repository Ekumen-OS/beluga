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

#include <gmock/gmock.h>

#include <beluga/algorithm/distance_map.hpp>

namespace {

auto array_distance(std::size_t first, std::size_t second) {
  return std::abs(static_cast<intmax_t>(first) - static_cast<intmax_t>(second));
}

auto make_neighbors_function(std::size_t size) {
  return [=](std::size_t index) {
    auto result = std::vector<std::size_t>{};
    if (index > 0) {
      result.push_back(index - 1);
    }
    if (index < size - 1) {
      result.push_back(index + 1);
    }
    return result;
  };
}

TEST(DistanceMap, None) {
  auto map = std::vector<bool>{};
  auto distance_map = nearest_obstacle_distance_map(map, array_distance, make_neighbors_function(6));
  ASSERT_EQ(distance_map.size(), 0);
}

TEST(DistanceMap, Empty) {
  auto map = std::array<bool, 6>{false, false, false, false, false, false};
  auto distance_map = nearest_obstacle_distance_map(map, array_distance, make_neighbors_function(6));
  ASSERT_THAT(distance_map, testing::ElementsAre(0, 0, 0, 0, 0, 0));
}

TEST(DistanceMap, Full) {
  auto map = std::array<bool, 6>{true, true, true, true, true, true};
  auto distance_map = nearest_obstacle_distance_map(map, array_distance, make_neighbors_function(6));
  ASSERT_THAT(distance_map, testing::ElementsAre(0, 0, 0, 0, 0, 0));
}

TEST(DistanceMap, Case1) {
  auto map = std::array<bool, 6>{false, true, false, false, false, true};
  auto distance_map = nearest_obstacle_distance_map(map, array_distance, make_neighbors_function(6));
  ASSERT_THAT(distance_map, testing::ElementsAre(1, 0, 1, 2, 1, 0));
}

TEST(DistanceMap, Case2) {
  auto map = std::array<bool, 6>{true, true, false, false, false, false};
  auto distance_map = nearest_obstacle_distance_map(map, array_distance, make_neighbors_function(6));
  ASSERT_THAT(distance_map, testing::ElementsAre(0, 0, 1, 2, 3, 4));
}

TEST(DistanceMap, Case3) {
  auto map = std::array<bool, 6>{false, false, false, false, false, true};
  auto distance_map = nearest_obstacle_distance_map(map, array_distance, make_neighbors_function(6));
  ASSERT_THAT(distance_map, testing::ElementsAre(5, 4, 3, 2, 1, 0));
}

}  // namespace

// Copyright 2023 Ekumen, Inc.
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

#include <gtest/gtest.h>

#include <cstddef>
#include <optional>
#include <vector>

#include "beluga/sensor/data/value_grid.hpp"

namespace {

TEST(LinearGrid2, InvalidSize) {
  constexpr std::size_t kWidth = 4;
  ASSERT_DEBUG_DEATH(beluga::ValueGrid2<bool>({{true, false, true, false, false}}, kWidth), "Assertion");
}

TEST(LinearGrid2, Indices) {
  constexpr std::size_t kWidth = 4;
  const auto grid = beluga::ValueGrid2<bool>{{{true, false, true, false, false, true, false, true}}, kWidth};

  EXPECT_EQ(grid.index_at(0, 0), 0);
  EXPECT_EQ(grid.index_at(3, 1), 7);

  EXPECT_EQ(grid.index_at(Eigen::Vector2i(0, 0)), 0);
  EXPECT_EQ(grid.index_at(Eigen::Vector2i(3, 1)), 7);
}

TEST(LinearGrid2, CoordinatesAtIndex) {
  constexpr std::size_t kWidth = 4;
  constexpr std::size_t kResolution = 1;
  const auto grid =
      beluga::ValueGrid2<bool>{{{true, false, true, false, false, true, false, true}}, kWidth, kResolution};

  EXPECT_EQ(grid.coordinates_at(0), Eigen::Vector2d(0.5, 0.5));
  EXPECT_EQ(grid.coordinates_at(3), Eigen::Vector2d(3.5, 0.5));
  EXPECT_EQ(grid.coordinates_at(5), Eigen::Vector2d(1.5, 1.5));
}

TEST(LinearGrid2, DataAtIndex) {
  constexpr std::size_t kWidth = 4;
  constexpr std::size_t kResolution = 1;
  const auto grid =
      beluga::ValueGrid2<bool>{{{true, false, true, false, false, true, false, true}}, kWidth, kResolution};

  EXPECT_TRUE(grid.data_at(0).value());
  EXPECT_FALSE(grid.data_at(3).value());
  EXPECT_TRUE(grid.data_at(5).value());
  EXPECT_EQ(grid.data_at(10), std::nullopt);
}

}  // namespace

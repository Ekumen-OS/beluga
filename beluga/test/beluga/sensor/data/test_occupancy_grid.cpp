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

#include <gmock/gmock.h>

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "beluga/test/static_occupancy_grid.hpp"

#include <range/v3/range/conversion.hpp>

#include <Eigen/Core>
#include <sophus/se2.hpp>

namespace {

using beluga::testing::StaticOccupancyGrid;

TEST(OccupancyGrid2, FreeAtCell) {
  constexpr double kResolution = 1.;

  const auto grid = StaticOccupancyGrid<5, 5>{
      {false, false, false, false, true,  false, false, false, true,  false, false, false, true,
       false, false, false, true,  false, false, false, true,  false, false, false, false},
      kResolution};

  EXPECT_TRUE(grid.free_at(0));
  EXPECT_FALSE(grid.free_at(8));
  EXPECT_FALSE(grid.free_at(2, 2));
  EXPECT_TRUE(grid.free_at(3, 2));
  EXPECT_FALSE(grid.free_at(Eigen::Vector2i(0, 4)));
  EXPECT_TRUE(grid.free_at(Eigen::Vector2i(1, 4)));
}

TEST(OccupancyGrid2, FreeNearCell) {
  constexpr double kResolution = 1.;

  const auto grid = StaticOccupancyGrid<5, 5>{
      {false, false, false, false, true,  false, false, false, true,  false, false, false, true,
       false, false, false, true,  false, false, false, true,  false, false, false, false},
      kResolution};

  EXPECT_FALSE(grid.free_near(3.25, 1.75));
  EXPECT_TRUE(grid.free_near(4, 4.25));
  EXPECT_FALSE(grid.free_near(Eigen::Vector2d(3.25, 1.75)));
  EXPECT_TRUE(grid.free_near(Eigen::Vector2d(4, 4.25)));
}

TEST(OccupancyGrid2, GlobalCoordinatesAtCell) {
  constexpr double kResolution = 1.;
  const auto origin = Sophus::SE2d{Sophus::SO2d{Sophus::Constants<double>::pi() / 2.}, Eigen::Vector2d{1., 1.}};
  const auto grid = StaticOccupancyGrid<5, 5>{
      {false, false, false, false, true,  false, false, false, true,  false, false, false, true,
       false, false, false, true,  false, false, false, true,  false, false, false, false},
      kResolution,
      origin};

  constexpr auto kFrame = StaticOccupancyGrid<5, 5>::Frame::kGlobal;
  EXPECT_TRUE(grid.coordinates_at(grid.index_at(2, 2), kFrame).isApprox(Eigen::Vector2d(-1.5, 3.5)));
}

TEST(OccupancyGrid2, AllFreeCells) {
  const auto grid = StaticOccupancyGrid<2, 5>{{false, false, false, false, true, false, false, false, true, false}};

  const auto expected_free_cells = std::vector<std::size_t>{0, 1, 2, 3, 5, 6, 7, 9};
  ASSERT_THAT(grid.free_cells() | ranges::to<std::vector>, testing::Pointwise(testing::Eq(), expected_free_cells));
}

TEST(OccupancyGrid2, ObstacleData) {
  const auto grid = StaticOccupancyGrid<5, 2>{{false, false, false, false, true, false, false, false, true, false}};

// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=111118
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
  // Data and obstacle data are equivalent in this case
  ASSERT_THAT(grid.obstacle_data() | ranges::to<std::vector>, testing::Pointwise(testing::Eq(), grid.data()));
#pragma GCC diagnostic pop
}

TEST(OccupancyGrid2, GlobalCoordinatesForCells) {
  constexpr double kResolution = 1.;
  const auto origin = Sophus::SE2d{Sophus::SO2d{Sophus::Constants<double>::pi() / 2.}, Eigen::Vector2d{1., 1.}};
  const auto grid = StaticOccupancyGrid<2, 3>{{false, true, false, true, false, true}, kResolution, origin};

  constexpr auto kFrame = StaticOccupancyGrid<2, 3>::Frame::kGlobal;
  const auto coordinates = grid.coordinates_for(grid.free_cells(), kFrame) | ranges::to<std::vector>;
  EXPECT_EQ(coordinates.size(), 3);
  EXPECT_TRUE(coordinates[0].isApprox(Eigen::Vector2d{0.5, 1.5}));
  EXPECT_TRUE(coordinates[1].isApprox(Eigen::Vector2d{0.5, 3.5}));
  EXPECT_TRUE(coordinates[2].isApprox(Eigen::Vector2d{-0.5, 2.5}));
}

}  // namespace

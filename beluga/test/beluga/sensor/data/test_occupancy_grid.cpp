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

using beluga::PlainGridStorage;
using beluga::testing::StaticOccupancyGrid;

TEST(OccupancyGrid2, FreeAtCell) {
  constexpr double kResolution = 1.;

  const auto grid = StaticOccupancyGrid(
      PlainGridStorage(5, 5, {false, false, false, false, true,  false, false, false, true,  false, false, false, true,
                              false, false, false, true,  false, false, false, true,  false, false, false, false}),
      kResolution, Sophus::SE2d{});

  EXPECT_FALSE(grid.free_at(2, 2));
  EXPECT_TRUE(grid.free_at(3, 2));
  EXPECT_FALSE(grid.free_at(Eigen::Vector2i(0, 4)));
  EXPECT_TRUE(grid.free_at(Eigen::Vector2i(1, 4)));
}

TEST(OccupancyGrid2, FreeNearCell) {
  constexpr double kResolution = 1.;

  const auto grid = StaticOccupancyGrid(
      PlainGridStorage(5, 5, {false, false, false, false, true,  false, false, false, true,  false, false, false, true,
                              false, false, false, true,  false, false, false, true,  false, false, false, false}),
      kResolution, Sophus::SE2d{});

  EXPECT_FALSE(grid.free_near(3.25, 1.75));
  EXPECT_TRUE(grid.free_near(4, 4.25));
  EXPECT_FALSE(grid.free_near(Eigen::Vector2d(3.25, 1.75)));
  EXPECT_TRUE(grid.free_near(Eigen::Vector2d(4, 4.25)));
}

TEST(OccupancyGrid2, GlobalCoordinatesAtCell) {
  constexpr double kResolution = 1.;
  const auto origin = Sophus::SE2d{Sophus::SO2d{Sophus::Constants<double>::pi() / 2.}, Eigen::Vector2d{1., 1.}};
  const auto grid = StaticOccupancyGrid(
      PlainGridStorage(
          5, 5,
          {
              false, false, false, false, true,   //
              false, false, false, true,  false,  //
              false, false, true,  false, false,  //
              false, true,  false, false, false,  //
              true,  false, false, false, false   //
          }),
      kResolution, origin);

  constexpr auto kFrame = StaticOccupancyGrid::Frame::kGlobal;
  EXPECT_TRUE(grid.coordinates_at({2, 2}, kFrame).isApprox(Eigen::Vector2d(-1.5, 3.5)));
}

TEST(OccupancyGrid2, AllFreeCells) {
  const auto grid = StaticOccupancyGrid(
      PlainGridStorage(
          5, 2,
          {
              false, false, false, false, true,  //
              false, false, false, true, false   //
          }),
      1., Sophus::SE2d{});

  const auto expected_free_cells = std::vector<Eigen::Vector2i>{
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {0, 1}, {1, 1}, {2, 1}, {4, 1},
  };
  ASSERT_THAT(grid.free_cells() | ranges::to<std::vector>, testing::Pointwise(testing::Eq(), expected_free_cells));
}

TEST(OccupancyGrid2, ObstacleData) {
  const auto grid = StaticOccupancyGrid(
      PlainGridStorage(
          4, 5,
          {
              false, false, false, true,   //
              false, false, true,  false,  //
              true,  false, false, false,  //
              false, false, false, false,  //
              true,  false, false, true    //
          }),
      1., Sophus::SE2d{});

  const auto expected_obstacle_data = std::vector<Eigen::Vector2i>{
      {3, 0}, {2, 1}, {0, 2}, {3, 4}, {0, 4},
  };

  // Data and obstacle data are equivalent in this case
  ASSERT_THAT(
      grid.obstacle_data() | ranges::to<std::vector>, testing::UnorderedElementsAreArray(expected_obstacle_data));
}

TEST(OccupancyGrid2, GlobalCoordinatesForCells) {
  constexpr double kResolution = 1.;
  const auto origin = Sophus::SE2d{Sophus::SO2d{Sophus::Constants<double>::pi() / 2.}, Eigen::Vector2d{1., 1.}};
  const auto grid = StaticOccupancyGrid(
      PlainGridStorage(
          3, 2,
          {
              false, true, false,  //
              true, false, true,   //
          }),
      kResolution, origin);

  constexpr auto kFrame = StaticOccupancyGrid::Frame::kGlobal;
  const auto coordinates = grid.coordinates_for(grid.free_cells(), kFrame) | ranges::to<std::vector>;
  EXPECT_EQ(coordinates.size(), 3);
  EXPECT_TRUE(coordinates[0].isApprox(Eigen::Vector2d{0.5, 1.5}));
  EXPECT_TRUE(coordinates[1].isApprox(Eigen::Vector2d{0.5, 3.5}));
  EXPECT_TRUE(coordinates[2].isApprox(Eigen::Vector2d{-0.5, 2.5}));
}

}  // namespace

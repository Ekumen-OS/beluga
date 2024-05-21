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

#include <cmath>
#include <optional>

#include <sophus/common.hpp>
#include <sophus/se2.hpp>

#include "beluga/algorithm/raycasting.hpp"
#include "beluga/test/raycasting.hpp"
#include "beluga/test/static_occupancy_grid.hpp"

namespace beluga {

using testing::StaticOccupancyGrid;

TEST(Raycasting, Nominal) {
  constexpr double kResolution = 0.5;
  // Note that axes are:
  // Positive X -> Right
  // Positive Y -> Down

  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false},
    kResolution};
  // clang-format on

  constexpr double kMaxRange = 5.;

  {
    // Horizontal ray that hits the map boundary.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{0.5, 0.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    EXPECT_EQ(ray.cast(Sophus::SO2d{0.}), std::nullopt);
  }

  {
    // Horizontal ray that hits the occupied cell.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{0., 1.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    EXPECT_EQ(ray.cast(Sophus::SO2d{0.}), 1.);
  }

  {
    // Downwards ray that hits the map boundary.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{0., 1.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    const auto distance = ray.cast(Sophus::SO2d{Sophus::Constants<double>::pi() / 2.});
    EXPECT_EQ(distance, std::nullopt);
  }

  {
    // Start cell is occupied, should return 0.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{1., 1.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    const auto distance = ray.cast(Sophus::SO2d{Sophus::Constants<double>::pi() / 2.});
    EXPECT_EQ(distance, 0.);
  }

  {
    // Downwards ray that is limited by beam range.
    constexpr double kShortenedRange = 1.;
    const auto pose = Sophus::SE2d{Sophus::Constants<double>::pi() / 2., Eigen::Vector2d{0., 0.}};
    const auto ray = Ray2d{grid, pose, kShortenedRange};
    EXPECT_EQ(ray.cast(Sophus::SO2d{0}), std::nullopt);
  }

  {
    // Downwards ray that hits the occupied cell.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{1., 0.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    const auto distance = ray.cast(Sophus::SO2d{Sophus::Constants<double>::pi() / 2.});
    EXPECT_EQ(distance, 1.);
  }

  {
    // Diagonal ray that hits the occupied cell.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{0., 0.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    const auto distance = ray.cast(Sophus::SO2d{Sophus::Constants<double>::pi() / 4.});
    EXPECT_EQ(distance, std::sqrt(2));
  }
}

TEST(Raycasting, NonIdentityGridOrigin) {
  constexpr double kResolution = 0.5;

  const auto origin = Sophus::SE2d{Sophus::SO2d{-Sophus::Constants<double>::pi() / 4.}, Eigen::Vector2d{0.5, 0.}};
  // Note that axes are:
  // Positive X -> Diagonal downwards right
  // Positive Y -> Diagonal downwards left

  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false},
    kResolution, origin};
  // clang-format on

  constexpr double kMaxRange = 5.;

  {
    // Diagonal ray that hits the occupied cell.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{0.5, 0.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    const auto distance = ray.cast(Sophus::SO2d{0.});
    EXPECT_EQ(distance, std::sqrt(2));
  }
}

TEST(BaselineRaycasting, Nominal) {
  constexpr double kResolution = 0.5;
  // Note that axes are:
  // Positive X -> Right
  // Positive Y -> Down

  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false},
    kResolution};
  // clang-format on

  constexpr double kMaxRange = 5.;

  using Constants = Sophus::Constants<double>;

  {
    // Horizontal ray that hits the map boundary.
    const auto pose = Eigen::Vector2d{0.5, 0.};
    const auto source = grid.cell_near(pose);
    const auto target = grid.cell_near(pose + kMaxRange * Sophus::SO2d{0.}.unit_complex());
    EXPECT_EQ(beluga::testing::raycast(grid, source, target), std::nullopt);
  }

  {
    // Horizontal ray that hits the occupied cell.
    const auto pose = Eigen::Vector2d{0., 1.};
    const auto source = grid.cell_near(pose);
    const auto target = grid.cell_near(pose + kMaxRange * Sophus::SO2d{0.}.unit_complex());
    EXPECT_EQ(beluga::testing::raycast(grid, source, target), 1.);
  }

  {
    // Downwards ray that hits the map boundary.
    const auto pose = Eigen::Vector2d{0., 1.};
    const auto source = grid.cell_near(pose);
    const auto target = grid.cell_near(pose + kMaxRange * Sophus::SO2d{Constants::pi() / 2.}.unit_complex());
    EXPECT_EQ(beluga::testing::raycast(grid, source, target), std::nullopt);
  }

  {
    // Start cell is occupied, should return 0.
    const auto pose = Eigen::Vector2d{1., 1.};
    const auto source = grid.cell_near(pose);
    const auto target = grid.cell_near(pose + kMaxRange * Sophus::SO2d{Constants::pi() / 2.}.unit_complex());
    EXPECT_EQ(beluga::testing::raycast(grid, source, target), 0.);
  }

  {
    // Downwards ray that is limited by beam range.
    constexpr double kShortenedRange = 1.;
    const auto pose = Eigen::Vector2d{0., 0.};
    const auto source = grid.cell_near(pose);
    const auto target = grid.cell_near(pose + kShortenedRange * Sophus::SO2d{Constants::pi() / 2.}.unit_complex());
    EXPECT_EQ(beluga::testing::raycast(grid, source, target), std::nullopt);
  }

  {
    // Downwards ray that hits the occupied cell.
    const auto pose = Eigen::Vector2d{1., 0.};
    const auto source = grid.cell_near(pose);
    const auto target = grid.cell_near(pose + kMaxRange * Sophus::SO2d{Constants::pi() / 2.}.unit_complex());
    EXPECT_EQ(beluga::testing::raycast(grid, source, target), 1.);
  }

  {
    // Diagonal ray that hits the occupied cell.
    const auto pose = Eigen::Vector2d{0., 0.};
    const auto source = grid.cell_near(pose);
    const auto target = grid.cell_near(pose + kMaxRange * Sophus::SO2d{Constants::pi() / 4.}.unit_complex());
    EXPECT_EQ(beluga::testing::raycast(grid, source, target), std::sqrt(2));
  }
}

}  // namespace beluga

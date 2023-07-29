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

#include "beluga/sensor/data/regular_grid2_mixin.hpp"

#include <ciabatta/ciabatta.hpp>
#include <range/v3/range/conversion.hpp>
namespace {

template <typename Mixin>
class TestRegularGrid2Mixin : public Mixin {
 public:
  [[nodiscard]] static double resolution() { return 1.; }
};

using TestRegularGrid2 = ciabatta::mixin<TestRegularGrid2Mixin, beluga::RegularGrid2Mixin>;

TEST(RegularGrid2, NearestCells) {
  const auto grid = TestRegularGrid2{};

  EXPECT_EQ(grid.cell_near(0, 0), Eigen::Vector2i(0, 0));
  EXPECT_EQ(grid.cell_near(1.5, 0.5), Eigen::Vector2i(1, 0));

  EXPECT_EQ(grid.cell_near(Eigen::Vector2d(0, 0)), Eigen::Vector2i(0, 0));
  EXPECT_EQ(grid.cell_near(Eigen::Vector2d(1.5, 0.5)), Eigen::Vector2i(1, 0));
}

TEST(RegularGrid2, CoordinatesAtCell) {
  const auto grid = TestRegularGrid2{};

  EXPECT_TRUE(grid.coordinates_at(0, 0).isApprox(Eigen::Vector2d(0.5, 0.5)));
  EXPECT_TRUE(grid.coordinates_at(1, 0).isApprox(Eigen::Vector2d(1.5, 0.5)));

  EXPECT_TRUE(grid.coordinates_at(Eigen::Vector2i(0, 0)).isApprox(Eigen::Vector2d(0.5, 0.5)));
  EXPECT_TRUE(grid.coordinates_at(Eigen::Vector2i(1, 0)).isApprox(Eigen::Vector2d(1.5, 0.5)));
}

TEST(RegularGrid2, CoordinatesForCells) {
  const auto grid = TestRegularGrid2{};

  const auto cells = std::vector{Eigen::Vector2i(1, 0), Eigen::Vector2i(4, 1)};
  const auto coordinates = grid.coordinates_for(cells) | ranges::to<std::vector>;
  EXPECT_EQ(coordinates.size(), 2);
  EXPECT_TRUE(coordinates[0].isApprox(Eigen::Vector2d(1.5, 0.5)));
  EXPECT_TRUE(coordinates[1].isApprox(Eigen::Vector2d(4.5, 1.5)));
}

}  // namespace

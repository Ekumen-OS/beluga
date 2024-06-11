// Copyright 2024 Ekumen, Inc.
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
#include <map>
#include <optional>
#include <unordered_map>

#include <Eigen/Core>

#include "beluga/sensor/data/sparse_value_grid.hpp"

namespace beluga {

template <class T>
class SparseGridTests : public testing::Test {};

struct Less {
  bool operator()(const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) const {
    return (lhs.x() < rhs.x()) || (lhs.y() < rhs.y());
  }
};

struct Hasher {
  std::size_t operator()(const Eigen::Vector2i&) const { return 1; }
};

using SparseGridTestCases =
    testing::Types<std::unordered_map<Eigen::Vector2i, int, Hasher>, std::map<Eigen::Vector2i, int, Less>>;

TYPED_TEST_SUITE(SparseGridTests, SparseGridTestCases, );

TYPED_TEST(SparseGridTests, CanBeConstructedEmpty) {
  const TypeParam data;
  [[maybe_unused]] const beluga::SparseValueGrid2<TypeParam> grid{data, 0.5};
}

TYPED_TEST(SparseGridTests, Size) {
  const TypeParam data{
      {Eigen::Vector2i{1, 2}, 1},
      {Eigen::Vector2i{4, 2}, 1},
      {Eigen::Vector2i{2, 2}, 1},
      {Eigen::Vector2i{3, 2}, 1},
  };

  const beluga::SparseValueGrid2<TypeParam> grid{data, 0.5};
  ASSERT_EQ(grid.size(), 4);
}

TYPED_TEST(SparseGridTests, NotPresent) {
  const TypeParam data{};

  const beluga::SparseValueGrid2<TypeParam> grid{data, 0.5};

  ASSERT_EQ(grid.data_at(Eigen::Vector2i(2, 1)), std::nullopt);
  ASSERT_EQ(grid.data_near(Eigen::Vector2d(5, 3)), std::nullopt);
  ASSERT_EQ(grid.data_near(Eigen::Vector2d{5, 1}), std::nullopt);
}

TYPED_TEST(SparseGridTests, Resolution) {
  {
    const beluga::SparseValueGrid2<TypeParam> grid{TypeParam{}, 0.5};
    ASSERT_DOUBLE_EQ(grid.resolution(), 0.5);
  }
  {
    const beluga::SparseValueGrid2<TypeParam> grid{TypeParam{}, 0.8};
    ASSERT_DOUBLE_EQ(grid.resolution(), 0.8);
  }
}

TYPED_TEST(SparseGridTests, DataAccessing) {
  const TypeParam data{
      {Eigen::Vector2i{1, 2}, 1},
      {Eigen::Vector2i{4, 2}, 2},
      {Eigen::Vector2i{3, 2}, 3},
      {Eigen::Vector2i{2, 2}, 4},
  };
  // trivial case where resolution == 1.0
  {
    const beluga::SparseValueGrid2<TypeParam> grid{data, 1.0};
    auto data = grid.data_near(Eigen::Vector2d{1.0, 2.0});
    ASSERT_TRUE(data.has_value());
    ASSERT_EQ(data, 1);
  }
  {
    const beluga::SparseValueGrid2<TypeParam> grid{data, 1.0};
    auto data = grid.data_near(Eigen::Vector2d{4.0, 2.0});
    ASSERT_TRUE(data.has_value());
    ASSERT_EQ(data, 2);
  }
  {
    const beluga::SparseValueGrid2<TypeParam> grid{data, 0.5};
    auto data = grid.data_near(Eigen::Vector2d{1.0, 2.0} * 0.5);
    ASSERT_TRUE(data.has_value());
    ASSERT_EQ(data, 1);
  }
}

TYPED_TEST(SparseGridTests, AllAccessorMethodsAreEquivalent) {
  const TypeParam data{
      {Eigen::Vector2i{1, 2}, 1}, {Eigen::Vector2i{4, 2}, 2}, {Eigen::Vector2i{3, 2}, 3}, {Eigen::Vector2i{2, 2}, 4}};

  for (const auto resolution : {0.1, 0.5, 1.2, 1.5}) {
    const beluga::SparseValueGrid2<TypeParam> grid{data, resolution};
    for (const auto& [coordinates, value] : data) {
      const Eigen::Vector2d double_coords = coordinates.template cast<double>() * resolution;
      ASSERT_TRUE(grid.cell_near(double_coords).isApprox(coordinates));
      ASSERT_EQ(grid.data_near(double_coords), std::make_optional<int>(value));
      ASSERT_EQ(grid.data_at(grid.cell_near(double_coords)), std::make_optional<int>(value));
    }
  }
}

}  // namespace beluga

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

#include <vector>

#include <beluga/sensor/data/cache_friendly_grid_storage.hpp>
#include <beluga/sensor/data/cache_friendly_grid_storage_2.hpp>
#include <beluga/sensor/data/cache_friendly_grid_storage_3.hpp>
#include <beluga/sensor/data/cache_friendly_grid_storage_4.hpp>
#include "beluga/sensor/data/linear_grid_storage.hpp"

namespace beluga {

namespace {

template <class T>
struct GridStorageTests : public testing::Test {
  using grid_storage_type = T;
};

using CellType = int32_t;

using Implementations = testing::Types<
    LinearGridStorage<CellType>,              //
    CacheFriendlyGridStorage<CellType, 64>,   //
    CacheFriendlyGridStorage2<CellType, 64>,  //
    CacheFriendlyGridStorage3<CellType>,      //
    CacheFriendlyGridStorage4<CellType, 64>   //
    >;

TYPED_TEST_SUITE(GridStorageTests, Implementations, );

TYPED_TEST(GridStorageTests, ConstructionWithoutDataInitialization) {
  // test that if no content is provided, the grid is empty
  using grid_storage_type = typename TestFixture::grid_storage_type;
  auto uut = grid_storage_type(5, 5);
  const auto& const_uut = uut;
  ASSERT_EQ(uut.width(), 5);
  ASSERT_EQ(uut.height(), 5);

  for (int i = 0; i < uut.width(); ++i) {
    for (int j = 0; j < uut.height(); ++j) {
      ASSERT_EQ(uut.cell(i, j), 0);
      ASSERT_EQ(const_uut.cell(i, j), 0);
    }
  }
}

TYPED_TEST(GridStorageTests, ConstructionWithDataInitialization) {
  // test that if no content is provided, the grid is empty
  using grid_storage_type = typename TestFixture::grid_storage_type;
  auto uut = grid_storage_type(
      5, 5,
      std::initializer_list<int32_t>{
          0,  1,  2,  3,  4,   //
          5,  6,  7,  8,  9,   //
          10, 11, 12, 13, 14,  //
          15, 16, 17, 18, 19,  //
          20, 21, 22, 23, 24,  //
      });
  const auto& const_uut = uut;

  ASSERT_EQ(uut.width(), 5);
  ASSERT_EQ(uut.height(), 5);

  for (int y = 0; y < uut.height(); ++y) {
    for (int x = 0; x < uut.width(); ++x) {
      ASSERT_EQ(uut.cell(x, y), x + y * uut.width());
      ASSERT_EQ(const_uut.cell(x, y), x + y * uut.width());
    }
  }
}

TYPED_TEST(GridStorageTests, SetCellData) {
  // test that if no content is provided, the grid is empty
  auto uut = LinearGridStorage<int32_t>(5, 5);
  const auto& const_uut = uut;

  ASSERT_EQ(uut.width(), 5);
  ASSERT_EQ(uut.height(), 5);

  for (int i = 0; i < uut.width(); ++i) {
    for (int j = 0; j < uut.height(); ++j) {
      ASSERT_EQ(uut.cell(i, j), 0);
      ASSERT_EQ(const_uut.cell(i, j), 0);
      uut.cell(i, j) = 99;
      ASSERT_EQ(uut.cell(i, j), 99);
      ASSERT_EQ(const_uut.cell(i, j), 99);
    }
  }
}

}  // namespace
}  // namespace beluga

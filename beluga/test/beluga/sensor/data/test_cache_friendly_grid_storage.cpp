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
#include <vector>

#include "beluga/sensor/data/cache_friendly_grid_storage.hpp"

namespace beluga {

namespace {

TEST(CacheFriendlyGridStorageTest, SetCellData) {
  // test that if no content is provided, the grid is empty
  auto uut = CacheFriendlyGridStorage<int32_t, 16>(50, 50);
  const auto& const_uut = uut;

  ASSERT_EQ(uut.width(), 50);
  ASSERT_EQ(uut.height(), 50);

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

TEST(CacheFriendlyGridStorageTest, ColoredCacheLinesForSquareArrangement) {
  using ValueType = int32_t;
  const std::size_t kWidth = 22;
  const std::size_t kHeight = 14;
  const std::size_t kLineLen = 64;
  const std::size_t kTileSize = kLineLen / sizeof(ValueType);
  const std::size_t kTileSide = static_cast<std::size_t>(std::sqrt(static_cast<double>(kTileSize)));

  auto uut = CacheFriendlyGridStorage<ValueType, kLineLen>(kWidth, kHeight);

  ASSERT_EQ(uut.width(), kWidth);
  ASSERT_EQ(uut.height(), kHeight);

  [[maybe_unused]] const auto tile_index = [&](int x, int y) {
    return static_cast<ValueType>((x / kTileSide) + (y / kTileSide) * ((kWidth + kTileSide - 1) / kTileSide));
  };

  const auto buffer_width = ((kWidth + kTileSide - 1) / kTileSide) * kTileSide;
  const auto buffer_height = ((kHeight + kTileSide - 1) / kTileSide) * kTileSide;
  for (std::size_t i = 0; i < buffer_height * buffer_width; ++i) {
    uut.raw_data()[i] = static_cast<ValueType>(i / kTileSize);
  }

  auto expected_grid_contents = std::vector<ValueType>({
      0,  0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  3,  3,  4,  4,  4,  4,  5,  5,   //
      0,  0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  3,  3,  4,  4,  4,  4,  5,  5,   //
      0,  0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  3,  3,  4,  4,  4,  4,  5,  5,   //
      0,  0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  3,  3,  4,  4,  4,  4,  5,  5,   //
      6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  8,  9,  9,  9,  9,  10, 10, 10, 10, 11, 11,  //
      6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  8,  9,  9,  9,  9,  10, 10, 10, 10, 11, 11,  //
      6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  8,  9,  9,  9,  9,  10, 10, 10, 10, 11, 11,  //
      6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  8,  9,  9,  9,  9,  10, 10, 10, 10, 11, 11,  //
      12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 17, 17,  //
      12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 17, 17,  //
      12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 17, 17,  //
      12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 17, 17,  //
      18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 21, 22, 22, 22, 22, 23, 23,  //
      18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 21, 22, 22, 22, 22, 23, 23,  //
  });
}

TEST(CacheFriendlyGridStorageTest, ColoredCacheLinesForRectangularArrangement) {
  using ValueType = int32_t;
  const std::size_t kWidth = 22;
  const std::size_t kHeight = 14;
  const std::size_t kLineLen = 32;
  const std::size_t kItemsPerLine = kLineLen / sizeof(ValueType);
  const std::size_t kTileSize = kItemsPerLine * 2;
  const std::size_t kTileSide = static_cast<std::size_t>(std::sqrt(static_cast<double>(kTileSize)));

  auto uut = CacheFriendlyGridStorage<ValueType, kLineLen>(kWidth, kHeight);

  ASSERT_EQ(uut.width(), kWidth);
  ASSERT_EQ(uut.height(), kHeight);

  [[maybe_unused]] const auto tile_index = [&](int x, int y) {
    return static_cast<ValueType>((x / kTileSide) + (y / kTileSide) * ((kWidth + kTileSide - 1) / kTileSide));
  };

  const auto buffer_width = ((kWidth + kTileSide - 1) / kTileSide) * kTileSide;
  const auto buffer_height = ((kHeight + kTileSide - 1) / kTileSide) * kTileSide;
  for (std::size_t i = 0; i < buffer_height * buffer_width; ++i) {
    uut.raw_data()[i] = static_cast<ValueType>(i / kItemsPerLine);
  }

  auto expected_grid_contents = std::vector<ValueType>({
      0,  0,  1,  1,  2,  2,  2,  2,  4,  4,  5,  5,  6,  6,  6,  6,  8,  8,  9,  9,  10, 10,  //
      0,  0,  1,  1,  2,  2,  2,  2,  4,  4,  5,  5,  6,  6,  6,  6,  8,  8,  9,  9,  10, 10,  //
      0,  0,  1,  1,  3,  3,  3,  3,  4,  4,  5,  5,  7,  7,  7,  7,  8,  8,  9,  9,  11, 11,  //
      0,  0,  1,  1,  3,  3,  3,  3,  4,  4,  5,  5,  7,  7,  7,  7,  8,  8,  9,  9,  11, 11,  //
      12, 12, 12, 12, 14, 14, 15, 15, 16, 16, 16, 16, 18, 18, 19, 19, 20, 20, 20, 20, 22, 22,  //
      12, 12, 12, 12, 14, 14, 15, 15, 16, 16, 16, 16, 18, 18, 19, 19, 20, 20, 20, 20, 22, 22,  //
      13, 13, 13, 13, 14, 14, 15, 15, 17, 17, 17, 17, 18, 18, 19, 19, 21, 21, 21, 21, 22, 22,  //
      13, 13, 13, 13, 14, 14, 15, 15, 17, 17, 17, 17, 18, 18, 19, 19, 21, 21, 21, 21, 22, 22,  //
      24, 24, 25, 25, 26, 26, 26, 26, 28, 28, 29, 29, 30, 30, 30, 30, 32, 32, 33, 33, 34, 34,  //
      24, 24, 25, 25, 26, 26, 26, 26, 28, 28, 29, 29, 30, 30, 30, 30, 32, 32, 33, 33, 34, 34,  //
      24, 24, 25, 25, 27, 27, 27, 27, 28, 28, 29, 29, 31, 31, 31, 31, 32, 32, 33, 33, 35, 35,  //
      24, 24, 25, 25, 27, 27, 27, 27, 28, 28, 29, 29, 31, 31, 31, 31, 32, 32, 33, 33, 35, 35,  //
      36, 36, 36, 36, 38, 38, 39, 39, 40, 40, 40, 40, 42, 42, 43, 43, 44, 44, 44, 44, 46, 46,  //
      36, 36, 36, 36, 38, 38, 39, 39, 40, 40, 40, 40, 42, 42, 43, 43, 44, 44, 44, 44, 46, 46,  //
  });

  for (int y = 0; y < uut.height(); ++y) {
    for (int x = 0; x < uut.width(); ++x) {
      ASSERT_EQ(uut.cell(x, y), expected_grid_contents[x + y * kWidth]);
    }
  }
}

}  // namespace
}  // namespace beluga

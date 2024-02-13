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

#include <iterator>
#include <type_traits>

#include <gmock/gmock.h>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/stride.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga/circular_array.hpp>
#include <beluga/type_traits/tuple_traits.hpp>

namespace {

TEST(CircularArrayTest, Empty) {
  const auto array = beluga::CircularArray<int, 2>{};
  EXPECT_EQ(array.size(), 0);
  EXPECT_EQ(array.effective_size(), 0);
  EXPECT_EQ(array.max_size(), 2);
  EXPECT_TRUE(array.empty());
  EXPECT_FALSE(array.full());
}

TEST(CircularArrayTest, PartiallyFilled) {
  const auto array = beluga::CircularArray<int, 2>{{1}};
  EXPECT_EQ(array.size(), 1);
  EXPECT_EQ(array.effective_size(), 1);
  EXPECT_EQ(array.max_size(), 2);
  EXPECT_FALSE(array.empty());
  EXPECT_FALSE(array.full());
}

TEST(CircularArrayTest, Full) {
  const auto array = beluga::CircularArray<int, 2>{{1, 2}};
  EXPECT_EQ(array.size(), 2);
  EXPECT_EQ(array.effective_size(), 2);
  EXPECT_EQ(array.max_size(), 2);
  EXPECT_FALSE(array.empty());
  EXPECT_TRUE(array.full());
}

TEST(CircularArrayTest, Read) {
  const auto array = beluga::CircularArray<int, 5>{{1, 2, 3}};
  EXPECT_EQ(array.front(), 1);
  EXPECT_EQ(array.back(), 3);
  EXPECT_EQ(array[0], 1);
  EXPECT_EQ(array.at(0), 1);
  EXPECT_EQ(array[1], 2);
  EXPECT_EQ(array.at(1), 2);
  EXPECT_EQ(array[2], 3);
  EXPECT_EQ(array.at(2), 3);
  EXPECT_THROW({ (void)array.at(3); }, std::out_of_range);
  EXPECT_THAT(array, ::testing::ElementsAre(1, 2, 3));
}

TEST(CircularArrayTest, ReverseLayoutRead) {
  constexpr auto kFlags = beluga::CircularArrayFeatureFlags::kLayoutReversal;
  const auto array = beluga::CircularArray<int, 5, kFlags>{{1, 2, 3}};
  EXPECT_EQ(array.front(), 1);
  EXPECT_EQ(array.back(), 3);
  EXPECT_EQ(array[0], 1);
  EXPECT_EQ(array.at(0), 1);
  EXPECT_EQ(array[1], 2);
  EXPECT_EQ(array.at(1), 2);
  EXPECT_EQ(array[2], 3);
  EXPECT_EQ(array.at(2), 3);
  EXPECT_THROW({ (void)array.at(3); }, std::out_of_range);
  EXPECT_THAT(array, ::testing::ElementsAre(1, 2, 3));
}

TEST(CircularArrayTest, ExtrapolateOnRead) {
  constexpr auto kFlags = beluga::CircularArrayFeatureFlags::kExtrapolateOnRead;
  const auto array = beluga::CircularArray<int, 5, kFlags>{{1, 2, 3}};
  EXPECT_EQ(array.size(), 3);
  EXPECT_EQ(array.effective_size(), 5);
  EXPECT_EQ(array.max_size(), 5);
  EXPECT_THAT(array, ::testing::ElementsAre(1, 2, 3, 3, 3));
}

TEST(CircularArrayTest, Write) {
  auto array = beluga::CircularArray<int, 2>{};
  EXPECT_TRUE(array.empty());
  array.push_back(1);
  array << 2;
  EXPECT_TRUE(array.full());
  EXPECT_THAT(array, ::testing::ElementsAre(1, 2));
  array.pop_front();
  EXPECT_THAT(array, ::testing::ElementsAre(2));
  array.pop_front();
  EXPECT_TRUE(array.empty());
}

TEST(CircularArrayTest, ReverseLayoutWrite) {
  constexpr auto kFlags = beluga::CircularArrayFeatureFlags::kLayoutReversal;
  auto array = beluga::CircularArray<int, 2, kFlags>{};
  EXPECT_TRUE(array.empty());
  array.push_front(1);
  array << 2;
  EXPECT_TRUE(array.full());
  EXPECT_THAT(array, ::testing::ElementsAre(2, 1));
  array.pop_back();
  EXPECT_THAT(array, ::testing::ElementsAre(2));
  array.pop_back();
  EXPECT_TRUE(array.empty());
}

TEST(CircularArrayTest, FrontInsertWrite) {
  constexpr auto kFlags = beluga::CircularArrayFeatureFlags::kLayoutReversal;
  auto output_array = beluga::CircularArray<int, 6, kFlags>{};
  const auto input_array = beluga::CircularArray<int, 3, kFlags>{{1, 3, 2}};
  std::copy(input_array.begin(), input_array.end(), std::front_inserter(output_array));
  std::copy(input_array.rbegin(), input_array.rend(), std::front_inserter(output_array));
  EXPECT_THAT(output_array, ::testing::ElementsAre(1, 3, 2, 2, 3, 1));
}

TEST(CircularArrayTest, BackInsertWrite) {
  auto output_array = beluga::CircularArray<int, 6>{};
  const auto input_array = beluga::CircularArray<int, 3>{{1, 3, 2}};
  std::copy(input_array.begin(), input_array.end(), std::back_inserter(output_array));
  std::copy(input_array.rbegin(), input_array.rend(), std::back_inserter(output_array));
  EXPECT_THAT(output_array, ::testing::ElementsAre(1, 3, 2, 2, 3, 1));
}

TEST(CircularArrayTest, NoWriteWhenFull) {
  auto array = beluga::CircularArray<int, 2>{{1, 2}};
  EXPECT_THROW({ array.push_back(3); }, std::length_error);
}

TEST(CircularArrayTest, RolloverOnWrite) {
  constexpr auto kFlags = beluga::CircularArrayFeatureFlags::kRolloverOnWrite;
  auto array = beluga::CircularArray<int, 2, kFlags>{{1, 2}};
  array.push_back(3);
  EXPECT_THAT(array, ::testing::ElementsAre(2, 3));
}

TEST(CircularArrayTest, Fill) {
  auto array = beluga::CircularArray<int, 3>{{1}};
  EXPECT_THAT(array, ::testing::ElementsAre(1));
  array.fill(0);
  EXPECT_THAT(array, ::testing::ElementsAre(1, 0, 0));
}

TEST(CircularArrayTest, Swap) {
  using std::swap;
  constexpr auto kFlags =
      beluga::CircularArrayFeatureFlags::kLayoutReversal | beluga::CircularArrayFeatureFlags::kRolloverOnWrite;
  auto lhs = beluga::CircularArray<int, 3, kFlags>{{-1, -2, -3}};
  lhs.push_front(0);  // then becomes 0, -1, -2
  auto rhs = beluga::CircularArray<int, 3>{{1, 2, 3}};
  swap(lhs, rhs);
  EXPECT_THAT(lhs, ::testing::ElementsAre(3, 2, 1));
  EXPECT_THAT(rhs, ::testing::ElementsAre(-2, -1, 0));
}

TEST(CircularArrayTest, RangeLike) {
  static_assert(ranges::random_access_range<beluga::CircularArray<int, 5>>);
  const auto input_array = beluga::CircularArray<int, 5>{{1, 2, 3, 4, 5}};
  const auto output_array = input_array | ranges::views::stride(2) |
                            ranges::views::transform([](int x) { return x * x; }) |
                            ranges::to<beluga::CircularArray<int, 3>>;
  EXPECT_THAT(output_array, ::testing::ElementsAre(1, 9, 25));
}

TEST(CircularArrayTest, TupleLike) {
  static_assert(beluga::is_tuple_like_v<beluga::CircularArray<int, 3>>);
  const auto array = beluga::CircularArray<int, 3>{{1, 2, 3}};
  const auto& [a, b, c] = array;
  EXPECT_EQ(a, 1);
  EXPECT_EQ(b, 2);
  EXPECT_EQ(c, 3);
}

}  // namespace

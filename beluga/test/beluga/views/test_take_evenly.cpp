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
#include <gtest/gtest.h>

#include <list>
#include <vector>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/reverse.hpp>

#include "beluga/views/take_evenly.hpp"

namespace {

TEST(TakeEvenlyView, ConceptChecksFromContiguousRange) {
  auto input = std::array{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  auto output = beluga::views::take_evenly(input, 2);

  static_assert(ranges::common_range<decltype(input)>);
  static_assert(!ranges::common_range<decltype(output)>);

  static_assert(!ranges::viewable_range<decltype(input)>);
  static_assert(ranges::viewable_range<decltype(output)>);

  static_assert(ranges::forward_range<decltype(input)>);
  static_assert(ranges::forward_range<decltype(output)>);

  static_assert(ranges::sized_range<decltype(input)>);
  static_assert(ranges::sized_range<decltype(output)>);

  static_assert(ranges::bidirectional_range<decltype(input)>);
  static_assert(ranges::bidirectional_range<decltype(output)>);

  static_assert(ranges::random_access_range<decltype(input)>);
  static_assert(ranges::random_access_range<decltype(output)>);

  static_assert(ranges::contiguous_range<decltype(input)>);
  static_assert(!ranges::contiguous_range<decltype(output)>);

  static_assert(ranges::range<decltype(output)>);
  static_assert(ranges::semiregular<decltype(output)>);
  static_assert(ranges::enable_view<decltype(output)>);
}

TEST(TakeEvenlyView, ConceptChecksFromBidirectionalRange) {
  auto input = std::list{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  auto output = beluga::views::take_evenly(input, 2);

  static_assert(ranges::forward_range<decltype(input)>);
  static_assert(ranges::forward_range<decltype(output)>);

  static_assert(ranges::bidirectional_range<decltype(input)>);
  static_assert(ranges::bidirectional_range<decltype(output)>);

  static_assert(!ranges::random_access_range<decltype(input)>);
  static_assert(!ranges::random_access_range<decltype(output)>);
}

TEST(TakeEvenlyView, NoElementsTakeZero) {
  const auto input = std::vector<int>{};
  auto output = input | beluga::views::take_evenly(0);
  ASSERT_EQ(output.size(), 0);
  ASSERT_EQ(ranges::to<std::vector>(output).size(), 0);
}

TEST(TakeEvenlyView, NoElements) {
  const auto input = std::vector<int>{};
  auto output = input | beluga::views::take_evenly(1);
  ASSERT_EQ(output.size(), 0);
  ASSERT_EQ(ranges::to<std::vector>(output).size(), 0);
}

TEST(TakeEvenlyView, TakeZero) {
  const auto input = std::vector<int>{1, 2, 3, 4};
  auto output = input | beluga::views::take_evenly(0);
  ASSERT_EQ(output.size(), 0);
  ASSERT_EQ(ranges::to<std::vector>(output).size(), 0);
}

TEST(TakeEvenlyView, TakeOne) {
  const auto input = std::vector<int>{1, 2, 3, 4};
  auto output = input | beluga::views::take_evenly(1);
  ASSERT_EQ(output.size(), 1);
  ASSERT_THAT(ranges::to<std::vector>(output), testing::ElementsAre(1));
}

TEST(TakeEvenlyView, TakeAll) {
  const auto input = std::vector<int>{1, 2, 3, 4};
  auto output = input | beluga::views::take_evenly(10);
  ASSERT_EQ(output.size(), 4);
  ASSERT_THAT(ranges::to<std::vector>(output), testing::ElementsAre(1, 2, 3, 4));
}

TEST(TakeEvenlyView, TakeTwoFromFour) {
  const auto input = std::vector<int>{1, 2, 3, 4};
  auto output = input | beluga::views::take_evenly(2);
  ASSERT_EQ(output.size(), 2);
  ASSERT_THAT(ranges::to<std::vector>(output), testing::ElementsAre(1, 4));
}

TEST(TakeEvenlyView, TakeThreeFromFive) {
  const auto input = std::vector<int>{1, 2, 3, 4, 5};
  auto output = input | beluga::views::take_evenly(3);
  ASSERT_EQ(output.size(), 3);
  ASSERT_THAT(ranges::to<std::vector>(output), testing::ElementsAre(1, 3, 5));
}

TEST(TakeEvenlyView, TakeThreeFromSix) {
  const auto input = std::vector<int>{1, 2, 3, 4, 5, 6};
  auto output = input | beluga::views::take_evenly(3);
  ASSERT_EQ(output.size(), 3);
  ASSERT_THAT(ranges::to<std::vector>(output), testing::ElementsAre(1, 4, 6));
}

TEST(TakeEvenlyView, TakeThreeFromNine) {
  const auto input = std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9};
  auto output = input | beluga::views::take_evenly(3);
  ASSERT_EQ(output.size(), 3);
  ASSERT_THAT(ranges::to<std::vector>(output), testing::ElementsAre(1, 5, 9));
}

TEST(TakeEvenlyView, TakeThreeFromFour) {
  const auto input = std::vector<int>{1, 2, 3, 4};
  auto output = input | beluga::views::take_evenly(3);
  ASSERT_EQ(output.size(), 3);
  ASSERT_THAT(ranges::to<std::vector>(output), testing::ElementsAre(1, 3, 4));
}

TEST(TakeEvenlyView, TakeSixFromTen) {
  const auto input = std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  auto output = input | beluga::views::take_evenly(6);
  ASSERT_EQ(output.size(), 6);
  ASSERT_THAT(ranges::to<std::vector>(output), testing::ElementsAre(1, 3, 5, 7, 9, 10));
}

TEST(TakeEvenlyView, TakeFromGenerator) {
  auto output = ranges::views::iota(1, 6) | beluga::views::take_evenly(3);
  ASSERT_EQ(output.size(), 3);
  ASSERT_THAT(ranges::to<std::vector>(output), testing::ElementsAre(1, 3, 5));
}

TEST(TakeEvenlyView, RandomAccess) {
  const auto input = std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9};
  auto output = input | beluga::views::take_evenly(3);
  ASSERT_EQ(output.size(), 3);
  ASSERT_EQ(output[0], 1);
  ASSERT_EQ(output[1], 5);
  ASSERT_EQ(output[2], 9);
}

TEST(TakeEvenlyView, RandomAccessReverse) {
  const auto input = std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9};
  auto output = input | beluga::views::take_evenly(3) | ranges::views::reverse;
  ASSERT_EQ(output.size(), 3);
  ASSERT_EQ(output[0], 9);
  ASSERT_EQ(output[1], 5);
  ASSERT_EQ(output[2], 1);
}

}  // namespace

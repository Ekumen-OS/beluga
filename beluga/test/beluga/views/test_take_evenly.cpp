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

#include <vector>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/iota.hpp>

#include "beluga/views/take_evenly.hpp"

namespace {

TEST(TakeEvenlyView, NoElementsTakeZero) {
  const auto input = std::vector<int>{};
  const auto output = input | beluga::views::take_evenly(0) | ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 0);
}

TEST(TakeEvenlyView, NoElements) {
  const auto input = std::vector<int>{};
  const auto output = input | beluga::views::take_evenly(1) | ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 0);
}

TEST(TakeEvenlyView, TakeZero) {
  const auto input = std::vector<int>{1, 2, 3, 4};
  const auto output = input | beluga::views::take_evenly(0) | ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 0);
}

TEST(TakeEvenlyView, TakeOne) {
  const auto input = std::vector<int>{1, 2, 3, 4};
  const auto output = input | beluga::views::take_evenly(1) | ranges::to<std::vector>;
  ASSERT_THAT(output, testing::ElementsAre(1));
}

TEST(TakeEvenlyView, TakeAll) {
  const auto input = std::vector<int>{1, 2, 3, 4};
  const auto output = input | beluga::views::take_evenly(10) | ranges::to<std::vector>;
  ASSERT_THAT(output, testing::ElementsAre(1, 2, 3, 4));
}

TEST(TakeEvenlyView, TakeTwoFromFour) {
  const auto input = std::vector<int>{1, 2, 3, 4};
  const auto output = input | beluga::views::take_evenly(2) | ranges::to<std::vector>;
  ASSERT_THAT(output, testing::ElementsAre(1, 4));
}

TEST(TakeEvenlyView, TakeThreeFromFive) {
  const auto input = std::vector<int>{1, 2, 3, 4, 5};
  const auto output = input | beluga::views::take_evenly(3) | ranges::to<std::vector>;
  ASSERT_THAT(output, testing::ElementsAre(1, 3, 5));
}

TEST(TakeEvenlyView, TakeThreeFromSix) {
  const auto input = std::vector<int>{1, 2, 3, 4, 5, 6};
  const auto output = input | beluga::views::take_evenly(3) | ranges::to<std::vector>;
  ASSERT_THAT(output, testing::ElementsAre(1, 4, 6));
}

TEST(TakeEvenlyView, TakeThreeFromNine) {
  const auto input = std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9};
  const auto output = input | beluga::views::take_evenly(3) | ranges::to<std::vector>;
  ASSERT_THAT(output, testing::ElementsAre(1, 5, 9));
}

TEST(TakeEvenlyView, TakeThreeFromFour) {
  const auto input = std::vector<int>{1, 2, 3, 4};
  const auto output = input | beluga::views::take_evenly(3) | ranges::to<std::vector>;
  ASSERT_THAT(output, testing::ElementsAre(1, 3, 4));
}

TEST(TakeEvenlyView, TakeSixFromTen) {
  const auto input = std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  const auto output = input | beluga::views::take_evenly(6) | ranges::to<std::vector>;
  ASSERT_THAT(output, testing::ElementsAre(1, 3, 5, 7, 9, 10));
}

TEST(TakeEvenlyView, TakeFromGenerator) {
  const auto output = ranges::views::iota(1, 6) | beluga::views::take_evenly(3) | ranges::to<std::vector>;
  ASSERT_THAT(output, testing::ElementsAre(1, 3, 5));
}

}  // namespace

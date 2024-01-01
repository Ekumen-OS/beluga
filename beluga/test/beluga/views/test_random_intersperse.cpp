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

#include <gmock/gmock.h>

#include "beluga/views/random_intersperse.hpp"

#include <range/v3/algorithm/count.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/take.hpp>

namespace {

TEST(RandomIntersperseView, ConceptChecksFromContiguousRange) {
  auto input = std::array{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  auto output = beluga::views::random_intersperse(input, []() { return 0; });

  static_assert(ranges::common_range<decltype(input)>);
  static_assert(!ranges::common_range<decltype(output)>);

  static_assert(!ranges::viewable_range<decltype(input)>);
  static_assert(ranges::viewable_range<decltype(output)>);

  static_assert(ranges::forward_range<decltype(input)>);
  static_assert(ranges::forward_range<decltype(output)>);

  static_assert(ranges::sized_range<decltype(input)>);
  static_assert(!ranges::sized_range<decltype(output)>);

  static_assert(ranges::bidirectional_range<decltype(input)>);
  static_assert(!ranges::bidirectional_range<decltype(output)>);

  static_assert(ranges::random_access_range<decltype(input)>);
  static_assert(!ranges::random_access_range<decltype(output)>);

  static_assert(ranges::contiguous_range<decltype(input)>);
  static_assert(!ranges::contiguous_range<decltype(output)>);
}

TEST(RandomIntersperseView, ConceptChecksFromInfiniteRange) {
  auto input = ranges::views::generate([]() { return 1; });
  auto output = beluga::views::random_intersperse(input, []() { return 0; });

  static_assert(!ranges::common_range<decltype(input)>);
  static_assert(!ranges::common_range<decltype(output)>);

  static_assert(ranges::viewable_range<decltype(input)>);
  static_assert(ranges::viewable_range<decltype(output)>);

  static_assert(!ranges::forward_range<decltype(input)>);
  static_assert(!ranges::forward_range<decltype(output)>);

  static_assert(!ranges::sized_range<decltype(input)>);
  static_assert(!ranges::sized_range<decltype(output)>);

  static_assert(!ranges::bidirectional_range<decltype(input)>);
  static_assert(!ranges::bidirectional_range<decltype(output)>);

  static_assert(!ranges::random_access_range<decltype(input)>);
  static_assert(!ranges::random_access_range<decltype(output)>);

  static_assert(!ranges::contiguous_range<decltype(input)>);
  static_assert(!ranges::contiguous_range<decltype(output)>);
}

TEST(RandomIntersperseView, GuaranteedIntersperseFirstElement) {
  auto input = std::array{10, 20, 30};
  auto output = input | beluga::views::random_intersperse([i = 0]() mutable { return i++; }, 1.0);
  auto it = ranges::begin(output);
  ASSERT_EQ(*it, 10);  // The first element is always from the input range
}

TEST(RandomIntersperseView, GuaranteedIntersperseDoubleDereference) {
  auto input = std::array{10, 20, 30};
  auto output = input | beluga::views::random_intersperse([i = 0]() mutable { return i++; }, 1.0);
  auto it = ranges::begin(output);
  ++it;
  ASSERT_EQ(*it, 0);
  ASSERT_EQ(*it, 0);
  ++it;
  ASSERT_EQ(*it, 1);
}

TEST(RandomIntersperseView, GuaranteedIntersperseTakeFive) {
  auto input = std::array{10, 20, 30};
  auto output = input |                                                       //
                beluga::views::random_intersperse([]() { return 4; }, 1.0) |  //
                ranges::views::take(5) |                                      //
                ranges::to<std::vector>;
  ASSERT_EQ(ranges::size(output), 5);
  ASSERT_THAT(output, testing::ElementsAre(10, 4, 4, 4, 4));
}

TEST(RandomIntersperseView, ZeroProbabilityIntersperseTakeFive) {
  auto input = std::array{10, 20, 30};
  auto output = input |                                                       //
                beluga::views::random_intersperse([]() { return 4; }, 0.0) |  //
                ranges::views::take(5) |                                      //
                ranges::to<std::vector>;
  ASSERT_EQ(ranges::size(output), 3);
  ASSERT_THAT(output, testing::ElementsAre(10, 20, 30));
}

TEST(RandomIntersperseView, ZeroProbabilityMultipass) {
  auto input = std::array{10, 20, 30};
  auto output = input |                                                       //
                beluga::views::random_intersperse([]() { return 4; }, 0.0) |  //
                ranges::views::take(3);
  ASSERT_THAT(output | ranges::to<std::vector>, testing::ElementsAre(10, 20, 30));
  ASSERT_THAT(output | ranges::to<std::vector>, testing::ElementsAre(10, 20, 30));
}

class RandomIntersperseViewWithParam : public ::testing::TestWithParam<double> {};

TEST_P(RandomIntersperseViewWithParam, TestPercentage) {
  const double expected_p = GetParam();
  const int size = 10'000;
  auto output = ranges::views::iota(1, size + 1) | beluga::views::random_intersperse([]() { return 0; }, expected_p);
  const double count = static_cast<double>(ranges::count(output, 0));
  const double actual_p = count / (size + count);
  ASSERT_NEAR(expected_p, actual_p, 0.01);
}

INSTANTIATE_TEST_SUITE_P(
    RandomIntersperseViewParams,
    RandomIntersperseViewWithParam,
    testing::Values(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9));

}  // namespace

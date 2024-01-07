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

#include "beluga/views/take_while_kld.hpp"

#include <range/v3/view/empty.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/intersperse.hpp>

namespace {

inline constexpr auto identity = [](auto&& t) noexcept { return std::forward<decltype(t)>(t); };

TEST(TakeWhileKld, ConceptChecksFromContiguousRange) {
  const std::size_t min = 0;
  const std::size_t max = 1200;
  const double epsilon = 0.05;
  auto input = std::array{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  auto output = beluga::views::take_while_kld(input, identity, min, max, epsilon);

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

TEST(TakeWhileKld, ConceptChecksFromInfiniteRange) {
  const std::size_t min = 0;
  const std::size_t max = 1200;
  const double epsilon = 0.05;
  auto input = ranges::views::generate([]() { return 1; });
  auto output = beluga::views::take_while_kld(input, identity, min, max, epsilon);

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

class KldConditionWithParam : public ::testing::TestWithParam<std::tuple<double, std::size_t, std::size_t>> {};

auto GenerateDistinctHashes(std::size_t count) {
  return ranges::views::generate([count, hash = 0UL]() mutable {
    if (hash < count) {
      ++hash;
    }
    return hash;
  });
}

TEST_P(KldConditionWithParam, Minimum) {
  const std::size_t cluster_count = std::get<1>(GetParam());
  const std::size_t min = 1'000;
  const double epsilon = 0.01;
  const double kld_k = 0.95;
  auto output = GenerateDistinctHashes(cluster_count) |  //
                ranges::views::take_while(beluga::kld_condition(min, epsilon, kld_k));
  ASSERT_GE(ranges::distance(output), min);
}

TEST_P(KldConditionWithParam, Limit) {
  const double kld_k = std::get<0>(GetParam());
  const std::size_t cluster_count = std::get<1>(GetParam());
  const std::size_t expected_count = std::get<2>(GetParam());
  const std::size_t min = 0;
  const double epsilon = 0.01;
  auto output = GenerateDistinctHashes(cluster_count) |  //
                ranges::views::take_while(beluga::kld_condition(min, epsilon, kld_k));
  ASSERT_EQ(ranges::distance(output), expected_count);
}

constexpr double kPercentile90th = 1.28155156327703;
constexpr double kPercentile99th = 2.32634787735669;

INSTANTIATE_TEST_SUITE_P(
    KldPairs,
    KldConditionWithParam,
    testing::Values(
        std::make_tuple(kPercentile90th, 3, 228),
        std::make_tuple(kPercentile90th, 4, 311),
        std::make_tuple(kPercentile90th, 5, 388),
        std::make_tuple(kPercentile90th, 6, 461),
        std::make_tuple(kPercentile90th, 7, 531),
        std::make_tuple(kPercentile90th, 100, 5871),
        std::make_tuple(kPercentile99th, 3, 462),
        std::make_tuple(kPercentile99th, 4, 569),
        std::make_tuple(kPercentile99th, 5, 666),
        std::make_tuple(kPercentile99th, 6, 756),
        std::make_tuple(kPercentile99th, 7, 843),
        std::make_tuple(kPercentile99th, 100, 6733)));

TEST(TakeWhileKld, TakeZero) {
  const std::size_t min = 2;
  const std::size_t max = 3;
  const double epsilon = 0.1;
  auto output = ranges::views::empty<std::size_t> |  //
                beluga::views::take_while_kld(identity, min, max, epsilon);
  ASSERT_EQ(ranges::distance(output), 0);
}

TEST(TakeWhileKld, TakeMaximum) {
  const std::size_t min = 200;
  const std::size_t max = 1200;
  const double epsilon = 0.05;
  auto output = ranges::views::generate([]() { return 1UL; }) |  //
                beluga::views::take_while_kld(identity, min, max, epsilon);
  ASSERT_EQ(ranges::distance(output), max);
}

TEST(TakeWhileKld, TakeLimit) {
  const std::size_t min = 0;
  const std::size_t max = 1200;
  const double epsilon = 0.05;
  auto output = ranges::views::generate([]() { return 1UL; }) |  //
                ranges::views::intersperse(2UL) |                //
                ranges::views::intersperse(3UL) |                //
                beluga::views::take_while_kld(identity, min, max, epsilon);
  ASSERT_EQ(ranges::distance(output), 135);
}

TEST(TakeWhileKld, TakeMinimum) {
  const std::size_t min = 200;
  const std::size_t max = 1200;
  const double epsilon = 0.05;
  auto output = ranges::views::generate([]() { return 1UL; }) |  //
                ranges::views::intersperse(2UL) |                //
                ranges::views::intersperse(3UL) |                //
                beluga::views::take_while_kld(identity, min, max, epsilon);
  ASSERT_EQ(ranges::distance(output), min);
}

TEST(TakeWhileKld, FromParticleRange) {
  struct State {};
  auto input = std::array{std::make_tuple(State{}, beluga::Weight(2.0))};
  const std::size_t min = 2;
  const std::size_t max = 3;
  const double epsilon = 0.1;
  const auto hasher = [](State) { return 42; };
  auto output = input | beluga::views::take_while_kld(hasher, min, max, epsilon);
  ASSERT_EQ(ranges::distance(output), 1);
  auto [state, weight] = *ranges::begin(output);
  ASSERT_EQ(weight, 2.0);
}

}  // namespace

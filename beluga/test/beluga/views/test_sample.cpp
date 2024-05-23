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

#include <array>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <range/v3/algorithm/count.hpp>
#include <range/v3/algorithm/find.hpp>
#include <range/v3/range/access.hpp>
#include <range/v3/range/concepts.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/range/dangling.hpp>
#include <range/v3/range/primitives.hpp>
#include <range/v3/view/const.hpp>
#include <range/v3/view/take_exactly.hpp>

#include "beluga/containers/tuple_vector.hpp"
#include "beluga/primitives.hpp"
#include "beluga/views/particles.hpp"
#include "beluga/views/sample.hpp"

namespace {

TEST(SampleView, FromEmptyRange) {
  auto input = std::vector<int>{};
  ASSERT_DEBUG_DEATH(beluga::views::sample(input), "Assertion");
}

TEST(SampleView, ConceptChecksFromContiguousRange) {
  auto input = std::array{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  auto output = beluga::views::sample(input);

  static_assert(ranges::common_range<decltype(input)>);
  static_assert(!ranges::common_range<decltype(output)>);

  static_assert(!ranges::viewable_range<decltype(input)>);
  static_assert(ranges::viewable_range<decltype(output)>);

  static_assert(ranges::forward_range<decltype(input)>);
  static_assert(!ranges::forward_range<decltype(output)>);

  static_assert(ranges::sized_range<decltype(input)>);
  static_assert(!ranges::sized_range<decltype(output)>);

  static_assert(ranges::bidirectional_range<decltype(input)>);
  static_assert(!ranges::bidirectional_range<decltype(output)>);

  static_assert(ranges::random_access_range<decltype(input)>);
  static_assert(!ranges::random_access_range<decltype(output)>);

  static_assert(ranges::contiguous_range<decltype(input)>);
  static_assert(!ranges::contiguous_range<decltype(output)>);

  static_assert(ranges::range<decltype(output)>);
  static_assert(ranges::semiregular<decltype(output)>);
  static_assert(ranges::enable_view<decltype(output)>);
}

TEST(SampleView, UniformDistributionSingleElement) {
  auto input = std::array{5};
  auto output = input | beluga::views::sample | ranges::views::take_exactly(20);
  ASSERT_EQ(ranges::count(output, 5), 20);
}

TEST(SampleView, DiscreteDistributionSingleElement) {
  auto input = std::array{5};
  auto weights = std::array{1.0};
  auto output = beluga::views::sample(input, weights) | ranges::views::take_exactly(20);
  ASSERT_EQ(ranges::count(output, 5), 20);
}

TEST(SampleView, DiscreteDistributionSingleElementFromParticleRange) {
  auto input = std::array{std::make_tuple(5, beluga::Weight(5.0))};
  // NOTE: We convert to std::vector because `sample` does not produce a forward range,
  // and thus does not support iterating over the whole sequence twice.
  auto output = input | beluga::views::sample | ranges::views::take_exactly(20) | ranges::to<std::vector>;
  ASSERT_EQ(ranges::count(output | beluga::views::states, 5), 20);
  ASSERT_EQ(ranges::count(output | beluga::views::weights, beluga::Weight(1.0)), 20);
}

TEST(SampleView, DiscreteDistributionWeightZero) {
  auto input = std::array{42, 5};
  auto weights = std::array{0.0, 1.0};
  auto output = beluga::views::sample(input, weights) | ranges::views::take_exactly(20);
  ASSERT_EQ(ranges::count(output, 5), 20);
}

TEST(SampleView, DoubleDereference) {
  auto engine = std::mt19937{std::random_device()()};
  auto input = std::array{10, 42, 39, 20, 50};
  auto output = beluga::views::sample(input, engine);
  auto it = ranges::begin(output);
  ++it;
  auto value = *it;
  ASSERT_EQ(value, *it);
  ASSERT_EQ(value, *it);
}

TEST(SampleView, NonBorrowedRange) {
  auto input = std::array{42};
  const auto create_view = [&]() { return input | beluga::views::sample; };
  auto it = ranges::find(create_view(), 42);
  static_assert(std::is_same_v<decltype(it), ranges::dangling>);  // the iterator is dangling since transform is not a
                                                                  // borrowed range
}

TEST(SampleView, EngineArgument) {
  auto engine = std::mt19937{std::random_device()()};
  auto input = std::array{5};
  auto weights = std::array{1.0};
  auto particles = std::array{std::make_tuple(5, beluga::Weight(1.0))};
  [[maybe_unused]] auto view1 = beluga::views::sample(engine);
  [[maybe_unused]] auto view2 = input | view1;
  [[maybe_unused]] auto view3 = particles | view1;
  [[maybe_unused]] auto view4 = beluga::views::sample(input, engine);
  [[maybe_unused]] auto view5 = beluga::views::sample(input, weights, engine);
  [[maybe_unused]] auto view6 = beluga::views::sample(particles, engine);
}

TEST(SampleView, DiscreteDistributionProbability) {
  const auto size = 100'000;

  const auto input = beluga::TupleVector<std::tuple<int, beluga::Weight>>{
      std::make_tuple(1, beluga::Weight(0.3)),  //
      std::make_tuple(2, beluga::Weight(0.1)),  //
      std::make_tuple(3, beluga::Weight(0.4)),  //
      std::make_tuple(4, beluga::Weight(0.2))};

  auto output = input |                  //
                ranges::views::const_ |  //
                beluga::views::sample |  //
                ranges::views::take_exactly(size);

  std::unordered_map<int, std::size_t> buckets;
  for (auto [value, weight] : output) {
    ++buckets[value];
    ASSERT_EQ(weight, 1.0);
  }

  ASSERT_EQ(ranges::size(buckets), 4);

  ASSERT_NEAR(static_cast<double>(buckets[1]) / size, 0.3, 0.01);
  ASSERT_NEAR(static_cast<double>(buckets[2]) / size, 0.1, 0.01);
  ASSERT_NEAR(static_cast<double>(buckets[3]) / size, 0.4, 0.01);
  ASSERT_NEAR(static_cast<double>(buckets[4]) / size, 0.2, 0.01);
}

TEST(SampleView, FromRandomDistributionFalse) {
  auto distribution = std::bernoulli_distribution{0.0};
  auto output = beluga::views::sample(distribution) | ranges::views::take_exactly(10);
  ASSERT_EQ(ranges::count(output, false), 10);
}

TEST(SampleView, FromRandomDistributionTrue) {
  auto distribution = std::bernoulli_distribution{1.0};
  auto output = beluga::views::sample(distribution) | ranges::views::take_exactly(10);
  ASSERT_EQ(ranges::count(output, true), 10);
}

}  // namespace

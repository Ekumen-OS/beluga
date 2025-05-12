// Copyright 2025 Ekumen, Inc.
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
#include <random>
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
#include "beluga/views/low_variance_sample.hpp"
#include "beluga/views/particles.hpp"

namespace {

TEST(LowVarianceSampleView, FromEmptyRange) {
  auto input = std::vector<int>{};
  ASSERT_DEBUG_DEATH(beluga::views::low_variance_sample(input), "Assertion");
}

TEST(LowVarianceSampleView, ConceptChecksFromContiguousRange) {
  auto input = std::array{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  auto output = beluga::views::low_variance_sample(input);

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

TEST(LowVarianceSampleView, UniformDistributionSingleElement) {
  auto input = std::array{5};
  auto output = input | beluga::views::low_variance_sample | ranges::views::take_exactly(20);
  ASSERT_EQ(ranges::count(output, 5), 20);
}

TEST(LowVarianceSampleView, DiscreteDistributionSingleElement) {
  auto input = std::array{5};
  auto weights = std::array{1.0};
  auto output = beluga::views::low_variance_sample(input, weights) | ranges::views::take_exactly(20);
  ASSERT_EQ(ranges::count(output, 5), 20);
}

TEST(LowVarianceSampleView, DiscreteDistributionSingleElementFromParticleRange) {
  auto input = std::array{std::make_tuple(5, beluga::Weight(5.0))};
  auto output = input | beluga::views::low_variance_sample | ranges::views::take_exactly(20) | ranges::to<std::vector>;
  ASSERT_EQ(ranges::count(output | beluga::views::states, 5), 20);
  ASSERT_EQ(ranges::count(output | beluga::views::weights, beluga::Weight(1.0)), 20);
}

TEST(LowVarianceSampleView, DoubleDereference) {
  auto engine = std::mt19937{std::random_device()()};
  auto input = std::array{10, 42, 39, 20, 50};
  auto output = beluga::views::low_variance_sample(input, engine);
  auto it = ranges::begin(output);
  ++it;
  auto value = *it;
  ASSERT_EQ(value, *it);
  ASSERT_EQ(value, *it);
}

TEST(LowVarianceSampleView, NonBorrowedRange) {
  auto input = std::array{42};
  const auto create_view = [&]() { return input | beluga::views::low_variance_sample; };
  auto it = ranges::find(create_view(), 42);
  static_assert(std::is_same_v<decltype(it), ranges::dangling>);
}

TEST(LowVarianceSampleView, EngineArgument) {
  auto engine = std::mt19937{std::random_device()()};
  auto input = std::array{5};
  auto weights = std::array{1.0};
  auto particles = std::array{std::make_tuple(5, beluga::Weight(1.0))};
  [[maybe_unused]] auto view1 = beluga::views::low_variance_sample(engine);
  [[maybe_unused]] auto view2 = input | view1;
  [[maybe_unused]] auto view3 = particles | view1;
  [[maybe_unused]] auto view4 = beluga::views::low_variance_sample(input, engine);
  [[maybe_unused]] auto view5 = beluga::views::low_variance_sample(input, weights, engine);
  [[maybe_unused]] auto view6 = beluga::views::low_variance_sample(particles, engine);
}

// Additional tests specific to low variance sampling properties

TEST(LowVarianceSampleView, DeterministicWithSameSeed) {
  const auto size = 100;

  const auto input = beluga::TupleVector<std::tuple<int, beluga::Weight>>{
      std::make_tuple(1, beluga::Weight(0.3)),  //
      std::make_tuple(2, beluga::Weight(0.7))   //
  };

  // First run
  auto engine1 = std::mt19937{12345};
  auto output1 =
      input | beluga::views::low_variance_sample(engine1) | ranges::views::take_exactly(size) | ranges::to<std::vector>;

  // Second run with same seed
  auto engine2 = std::mt19937{12345};
  auto output2 =
      input | beluga::views::low_variance_sample(engine2) | ranges::views::take_exactly(size) | ranges::to<std::vector>;

  // Results should be identical
  ASSERT_EQ(output1.size(), output2.size());
  for (std::size_t i = 0; i < output1.size(); ++i) {
    ASSERT_EQ(beluga::views::states(output1)[i], beluga::views::states(output2)[i]);
  }
}

TEST(LowVarianceSampleView, VerySmallWeights) {
  auto input = std::array{1, 2, 3};
  auto weights = std::array{1e-10, 1e-8, 1.0};
  auto output =
      beluga::views::low_variance_sample(input, weights) | ranges::views::take_exactly(100) | ranges::to<std::vector>;

  // Element 3 should dominate due to much larger weight
  auto count_3 = ranges::count(output, 3);
  ASSERT_GT(count_3, 95);  // Should be almost all element 3
}

TEST(LowVarianceSampleView, LargeRangePerformance) {
  // Test with larger input to verify performance characteristics
  std::vector<int> large_input(1000);
  std::iota(large_input.begin(), large_input.end(), 1);

  std::vector<double> uniform_weights(1000, 1.0 / 1000.0);

  auto engine = std::mt19937{42};
  auto output = beluga::views::low_variance_sample(large_input, uniform_weights, engine) |
                ranges::views::take_exactly(5000) | ranges::to<std::vector>;

  // Should complete without issues and produce correct number of samples
  ASSERT_EQ(output.size(), 5000);

  // All samples should be from the original range
  for (const auto& sample : output) {
    ASSERT_GE(sample, 1);
    ASSERT_LE(sample, 1000);
  }
}

}  // namespace

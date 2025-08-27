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

  std::vector<double> uniform_weights(large_input.size(), 1.0 / static_cast<double>(large_input.size()));

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

// Multinomial sampling for comparison - simple implementation
std::vector<int> multinomial_sample(
    const std::vector<int>& input,
    const std::vector<double>& weights,
    int num_samples,
    std::mt19937& engine) {
  std::discrete_distribution<> dist(weights.begin(), weights.end());
  std::vector<int> result;
  result.reserve(num_samples);

  for (int i = 0; i < num_samples; ++i) {
    result.push_back(input[dist(engine)]);
  }

  return result;
}

TEST(LowVarianceSampleView, LowerVarianceThanMultinomialSampling) {
  const auto num_samples = 1000;
  const auto num_trials = 200;

  auto input = std::vector<int>{1, 2, 3, 4};
  auto weights = std::vector<double>{0.1, 0.25, 0.35, 0.3};

  std::vector<double> lv_variances, multinomial_variances;

  for (int trial = 0; trial < num_trials; ++trial) {
    // Low variance sampling
    auto engine_lv = std::mt19937{static_cast<unsigned>(trial + 5000)};
    auto lv_output = beluga::views::low_variance_sample(input, weights, engine_lv) |
                     ranges::views::take_exactly(num_samples) | ranges::to<std::vector>;

    // Multinomial sampling
    auto engine_mult = std::mt19937{static_cast<unsigned>(trial + 5000)};  // Same seed
    auto mult_output = multinomial_sample(input, weights, num_samples, engine_mult);

    auto lv_count_1 = ranges::count(lv_output, 1);
    auto mult_count_1 = ranges::count(mult_output, 1);

    double lv_ratio = static_cast<double>(lv_count_1) / num_samples;
    double mult_ratio = static_cast<double>(mult_count_1) / num_samples;

    // Store squared deviation from expected (0.1)
    lv_variances.push_back(std::pow(lv_ratio - 0.1, 2));
    multinomial_variances.push_back(std::pow(mult_ratio - 0.1, 2));
  }

  // Calculate mean squared errors (approximation of variance)
  double lv_mse = std::accumulate(lv_variances.begin(), lv_variances.end(), 0.0) / num_trials;
  double mult_mse = std::accumulate(multinomial_variances.begin(), multinomial_variances.end(), 0.0) / num_trials;

  // Low variance sampling should have lower variance (MSE)
  ASSERT_LT(lv_mse, mult_mse);

  ASSERT_LT(lv_mse, mult_mse * 0.8);

  std::vector<double> lv_total_var, mult_total_var;

  for (int trial = 0; trial < std::min(50, num_trials); ++trial) {  // Subset for performance
    auto engine_lv = std::mt19937{static_cast<unsigned>(trial + 6000)};
    auto lv_output = beluga::views::low_variance_sample(input, weights, engine_lv) |
                     ranges::views::take_exactly(num_samples) | ranges::to<std::vector>;

    auto engine_mult = std::mt19937{static_cast<unsigned>(trial + 6000)};
    auto mult_output = multinomial_sample(input, weights, num_samples, engine_mult);

    // Calculate total variance across all elements
    double lv_var_sum = 0.0, mult_var_sum = 0.0;
    for (size_t i = 0; i < input.size(); ++i) {
      auto lv_count = ranges::count(lv_output, input[i]);
      auto mult_count = ranges::count(mult_output, input[i]);

      double lv_ratio = static_cast<double>(lv_count) / num_samples;
      double mult_ratio = static_cast<double>(mult_count) / num_samples;

      lv_var_sum += std::pow(lv_ratio - weights[i], 2);
      mult_var_sum += std::pow(mult_ratio - weights[i], 2);
    }

    lv_total_var.push_back(lv_var_sum);
    mult_total_var.push_back(mult_var_sum);
  }

  double lv_avg_total_var =
      std::accumulate(lv_total_var.begin(), lv_total_var.end(), 0.0) / static_cast<double>(lv_total_var.size());
  double mult_avg_total_var =
      std::accumulate(mult_total_var.begin(), mult_total_var.end(), 0.0) / static_cast<double>(mult_total_var.size());

  ASSERT_LT(lv_avg_total_var, mult_avg_total_var);
}

}  // namespace

// Copyright 2022 Ekumen, Inc.
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

#include <range/v3/view.hpp>

#include <beluga/algorithm/sampling.h>

namespace {

class RandomSelectWithParam : public ::testing::TestWithParam<double> {};

TEST_P(RandomSelectWithParam, Functional) {
  const double probability = GetParam();
  auto generator = std::mt19937{std::random_device()()};
  auto output =
      ranges::views::generate(beluga::random_select([]() { return 1; }, []() { return 2; }, generator, probability)) |
      ranges::views::take_exactly(1'000'000);

  std::size_t one_count = 0;
  std::size_t two_count = 0;
  for (auto&& value : output) {
    if (value == 1) {
      ++one_count;
    }
    if (value == 2) {
      ++two_count;
    }
  }
  ASSERT_NEAR(probability, static_cast<double>(one_count) / (one_count + two_count), 0.01);
}

INSTANTIATE_TEST_SUITE_P(Probabilities, RandomSelectWithParam, testing::Values(0.0, 0.2, 0.4, 0.6, 0.8, 1.0));

TEST(RandomSample, Functional) {
  const std::size_t samples = 1'000'000;
  auto values = std::array<int, 4>{1, 2, 3, 4};
  auto weights = std::array<double, 4>{0.1, 0.4, 0.3, 0.2};
  auto generator = std::mt19937{std::random_device()()};
  auto output =
      ranges::views::generate(beluga::random_sample(values, weights, generator)) | ranges::views::take_exactly(samples);

  auto counters = std::array<std::size_t, 4>{0, 0, 0, 0};
  for (auto&& value : output) {
    auto* it = std::find(std::begin(values), std::end(values), value);
    ASSERT_NE(it, std::end(values)) << "Random sample returned an unknown value.";
    auto& counter = *(std::begin(counters) + std::distance(std::begin(values), it));
    ++counter;
  }

  for (std::size_t i = 0; i < counters.size(); ++i) {
    ASSERT_NEAR(weights[i], static_cast<double>(counters[i]) / samples, 0.01);
  }
}

class KLDConditionWithParam : public ::testing::TestWithParam<std::tuple<double, std::size_t, std::size_t>> {};

TEST_P(KLDConditionWithParam, Minimum) {
  const std::size_t cluster_count = std::get<1>(GetParam());
  const std::size_t fixed_min_samples = 1'000;
  auto predicate = beluga::kld_condition(fixed_min_samples, 0.01, 0.95);
  std::size_t cluster = 0;
  for (std::size_t i = 0; i < fixed_min_samples - 1; ++i) {
    ASSERT_TRUE(predicate(cluster)) << "Stopped at " << i + 1 << " samples (Expected: " << fixed_min_samples << ").";
    if (cluster < cluster_count - 1) {
      ++cluster;
    }
  }
}

TEST_P(KLDConditionWithParam, Limit) {
  const double kld_k = std::get<0>(GetParam());
  const std::size_t cluster_count = std::get<1>(GetParam());
  const std::size_t min_samples = std::get<2>(GetParam());
  auto predicate = beluga::kld_condition(0, 0.01, kld_k);
  std::size_t cluster = 0;
  for (std::size_t i = 0; i < min_samples - 1; ++i) {
    ASSERT_TRUE(predicate(cluster)) << "Stopped at " << i + 1 << " samples (Expected: " << min_samples << ").";
    if (cluster < cluster_count - 1) {
      ++cluster;
    }
  }
  ASSERT_FALSE(predicate(cluster)) << "Didn't stop at " << min_samples << " samples.";
}

constexpr double kPercentile90th = 1.28155156327703;
constexpr double kPercentile99th = 2.32634787735669;

INSTANTIATE_TEST_SUITE_P(
    KLDPairs,
    KLDConditionWithParam,
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

}  // namespace

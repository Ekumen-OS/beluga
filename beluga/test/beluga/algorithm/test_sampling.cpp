// Copyright 2022-2023 Ekumen, Inc.
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

#include <beluga/algorithm/sampling.hpp>
#include <ciabatta/ciabatta.hpp>
#include <range/v3/algorithm/count.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/intersperse.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/take_while.hpp>

namespace {

using testing::_;
using testing::DoubleEq;
using testing::Each;
using testing::Return;
using testing::ReturnRef;

class RandomSelectWithParam : public ::testing::TestWithParam<double> {};

TEST_P(RandomSelectWithParam, Functional) {
  const double probability = GetParam();
  auto generator = std::mt19937{std::random_device()()};
  auto output = ranges::views::generate(
                    beluga::make_random_selector([]() { return 1; }, []() { return 2; }, generator, probability)) |
                ranges::views::take_exactly(1'000'000);
  auto one_count = static_cast<size_t>(ranges::count(output, 1));
  auto two_count = static_cast<size_t>(ranges::count(output, 2));
  ASSERT_NEAR(probability, static_cast<double>(one_count) / static_cast<double>(one_count + two_count), 0.01);
}

INSTANTIATE_TEST_SUITE_P(Probabilities, RandomSelectWithParam, testing::Values(0.0, 0.2, 0.4, 0.6, 0.8, 1.0));

template <class Range, class Values, class Weights>
void AssertWeights(Range&& range, const Values& values, const Weights& weights) {
  auto counters = std::vector<std::size_t>(weights.size(), 0);
  for (auto state : range) {
    auto it = std::find(std::begin(values), std::end(values), state);
    ASSERT_NE(it, std::end(values)) << "Unknown value: " << state;
    auto& counter = *(std::begin(counters) + std::distance(std::begin(values), it));
    ++counter;
  }
  for (auto [i, weight] : ranges::views::enumerate(weights)) {
    ASSERT_NEAR(weight, static_cast<double>(counters[i]) / static_cast<double>(range.size()), 0.01);
  }
}

TEST(RandomSample, Functional) {
  const std::size_t samples = 1'000'000;
  auto values = std::array<int, 4>{1, 2, 3, 4};
  auto weights = std::array<double, 4>{0.1, 0.4, 0.3, 0.2};
  auto generator = std::mt19937{std::random_device()()};
  auto output = ranges::views::generate(beluga::make_multinomial_sampler(values, weights, generator)) |
                ranges::views::take_exactly(samples);
  AssertWeights(output, values, weights);
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
  auto output = GenerateDistinctHashes(cluster_count) |
                ranges::views::take_while(beluga::kld_condition(1'000, 0.01, 0.95)) | ranges::to<std::vector>;
  ASSERT_GE(output.size(), 1'000);
}

TEST_P(KldConditionWithParam, Limit) {
  const double kld_k = std::get<0>(GetParam());
  const std::size_t cluster_count = std::get<1>(GetParam());
  const std::size_t min_samples = std::get<2>(GetParam());
  auto output = GenerateDistinctHashes(cluster_count) |
                ranges::views::take_while(beluga::kld_condition(0, 0.01, kld_k)) | ranges::to<std::vector>;
  ASSERT_EQ(output.size(), min_samples);
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

template <class Mixin>
class MockStorage : public Mixin {
 public:
  using particle_type = std::tuple<int, beluga::Weight>;

  template <class... Args>
  explicit MockStorage(Args&&... rest) : Mixin(std::forward<Args>(rest)...) {}

  MOCK_METHOD(int, make_random_state, (std::mt19937&));
  MOCK_METHOD(const std::vector<int>&, states, (), (const));
  MOCK_METHOD(const std::vector<double>&, weights, (), (const));
};

TEST(RandomStateGenerator, InstantiateAndCall) {
  auto generator = std::mt19937{std::random_device()()};
  auto instance = ciabatta::mixin<MockStorage, beluga::RandomStateGenerator>{};
  EXPECT_CALL(instance, make_random_state(_)).WillRepeatedly(Return(5));
  auto output = instance.generate_samples(generator) | ranges::views::take_exactly(50) | ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 50);
  ASSERT_THAT(output, Each(5));
}

TEST(NaiveSampler, Distribution) {
  auto generator = std::mt19937{std::random_device()()};
  auto states = std::vector<int>{1, 2, 3, 4};
  auto weights = std::vector<double>{0.3, 0.1, 0.4, 0.2};
  auto instance = ciabatta::mixin<MockStorage, beluga::NaiveSampler>{};
  EXPECT_CALL(instance, states()).WillOnce(ReturnRef(states));
  EXPECT_CALL(instance, weights()).WillOnce(ReturnRef(weights));
  auto output = instance.generate_samples_from_particles(generator) | ranges::views::take_exactly(100'000) |
                ranges::to<std::vector>;
  AssertWeights(output, states, weights);
}

TEST(AdaptiveSampler, Distribution) {
  auto generator = std::mt19937{std::random_device()()};
  auto states = std::vector<int>{1, 2, 3, 4};
  auto weights = std::vector<double>{0.3, 0.1, 0.4, 0.2};
  auto instance = ciabatta::mixin<MockStorage, beluga::AdaptiveSampler>{beluga::AdaptiveSamplerParam{0.001, 0.1}};
  EXPECT_CALL(instance, states()).WillOnce(ReturnRef(states));
  EXPECT_CALL(instance, weights()).WillOnce(ReturnRef(weights));
  auto output = instance.generate_samples_from_particles(generator) | ranges::views::take_exactly(100'000) |
                ranges::to<std::vector>;
  AssertWeights(output, states, weights);
}

class AdaptiveSamplerWithParam : public ::testing::TestWithParam<std::tuple<double, double, double>> {};

TEST_P(AdaptiveSamplerWithParam, RandomParticleCount) {
  const auto [initial_average_weight, final_average_weight, random_particle_probability] = GetParam();

  auto instance = ciabatta::mixin<MockStorage, beluga::AdaptiveSampler>{beluga::AdaptiveSamplerParam{0.001, 0.1}};

  // A "random" generated particle will be any particle with value 5
  EXPECT_CALL(instance, make_random_state(_)).WillRepeatedly(Return(5));
  auto random_particle_percentage = [](const auto& range) {
    // Count "random" generated particles by counting the number of 5's
    return static_cast<double>(ranges::count(range, 5)) / static_cast<double>(range.size());
  };

  auto states = std::vector<int>{1, 2, 3, 4};
  auto weights = std::vector<double>{0.0, 0.0, 0.0, 0.0};
  EXPECT_CALL(instance, states()).WillRepeatedly(ReturnRef(states));
  EXPECT_CALL(instance, weights()).WillRepeatedly(ReturnRef(weights));

  auto create_samples = [&instance]() {
    static auto generator = std::mt19937{std::random_device()()};
    return instance.generate_samples_from_particles(generator) | ranges::views::take_exactly(100'000) |
           ranges::to<std::vector>;
  };

  std::fill(weights.begin(), weights.end(), initial_average_weight);
  auto output = create_samples();
  ASSERT_EQ(0, random_particle_percentage(output));

  std::fill(weights.begin(), weights.end(), final_average_weight);
  output = create_samples();
  ASSERT_NEAR(random_particle_probability, random_particle_percentage(output), 0.01);

  // Once random particles are injected, reset filters
  output = create_samples();
  ASSERT_EQ(0, random_particle_percentage(output));
}

INSTANTIATE_TEST_SUITE_P(
    AdaptiveParams,
    AdaptiveSamplerWithParam,
    testing::Values(
        std::make_tuple(1.0, 1.5, 0.00),
        std::make_tuple(1.0, 2.0, 0.00),
        std::make_tuple(1.0, 0.5, 0.05),
        std::make_tuple(0.5, 0.1, 0.08),
        std::make_tuple(0.5, 0.0, 0.10)));

TEST(FixedLimiter, TakeSamplesLeavesUnitWeight) {
  auto instance = ciabatta::mixin<MockStorage, beluga::FixedLimiter>{beluga::FixedLimiterParam{1'200}};
  auto output = ranges::views::generate([]() { return 1; }) | instance.take_samples() |
                ranges::views::transform([](const auto& p) -> double { return std::get<1>(p); }) |
                ranges::to<std::vector>;
  ASSERT_THAT(output, Each(DoubleEq(1.0)));
}

TEST(FixedLimiter, TakeMaximum) {
  auto instance = ciabatta::mixin<MockStorage, beluga::FixedLimiter>{beluga::FixedLimiterParam{1'200}};
  auto output = ranges::views::generate([]() { return 1; }) | instance.take_samples() | ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 1'200);
}

template <class Mixin>
struct MockStorageWithCluster : public Mixin {
  using state_type = std::pair<double, double>;
  using particle_type = std::tuple<state_type, beluga::Weight, beluga::Cluster>;

  template <class... Args>
  explicit MockStorageWithCluster(Args&&... rest) : Mixin(std::forward<Args>(rest)...) {}
};

TEST(KldLimiter, TakeSamplesLeavesUnitWeight) {
  auto instance = ciabatta::mixin<MockStorage, beluga::FixedLimiter>{beluga::FixedLimiterParam{1'200}};
  auto output = ranges::views::generate([]() { return 1; }) | instance.take_samples() |
                ranges::views::transform([](const auto& p) -> double { return std::get<1>(p); }) |
                ranges::to<std::vector>;
  ASSERT_THAT(output, Each(DoubleEq(1.0)));
}

using state = std::pair<double, double>;

TEST(KldLimiter, TakeMaximum) {
  constexpr std::array kClusteringResolution{1., 1.};
  auto hasher = beluga::spatial_hash<state>{kClusteringResolution};
  auto instance = ciabatta::mixin<MockStorageWithCluster, ciabatta::curry<beluga::KldLimiter, state>::mixin>{
      beluga::KldLimiterParam<state>{200, 1'200, hasher, 0.05, 3.}};
  auto output = ranges::views::generate([]() { return std::make_pair(1., 1.); }) | instance.take_samples() |
                ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 1'200);
}

TEST(KldLimiter, TakeLimit) {
  constexpr std::array kClusteringResolution{0.05, 0.05};
  auto hasher = beluga::spatial_hash<state>{kClusteringResolution};
  auto instance = ciabatta::mixin<MockStorageWithCluster, ciabatta::curry<beluga::KldLimiter, state>::mixin>{
      beluga::KldLimiterParam<state>{0, 1'200, hasher, 0.05, 3.}};
  auto output = ranges::views::generate([]() { return std::make_pair(1., 1.); }) |
                ranges::views::intersperse(std::make_pair(2., 2.)) |
                ranges::views::intersperse(std::make_pair(3., 3.)) | instance.take_samples() | ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 135);
}

TEST(KldLimiter, TakeMinimum) {
  constexpr std::array kClusteringResolution{1., 1.};
  auto hasher = beluga::spatial_hash<state>{kClusteringResolution};
  auto instance = ciabatta::mixin<MockStorageWithCluster, ciabatta::curry<beluga::KldLimiter, state>::mixin>{
      beluga::KldLimiterParam<state>{200, 1'200, hasher, 0.05, 3.}};
  auto output = ranges::views::generate([]() { return std::make_pair(1., 1.); }) |
                ranges::views::intersperse(std::make_pair(2., 2.)) |
                ranges::views::intersperse(std::make_pair(3., 3.)) | instance.take_samples() | ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 200);
}

}  // namespace

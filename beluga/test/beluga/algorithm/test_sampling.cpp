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

#include <gmock/gmock.h>

#include <range/v3/algorithm.hpp>
#include <range/v3/view.hpp>

#include <beluga/algorithm/sampling.h>
#include <ciabatta/ciabatta.h>

namespace {

class RandomSelectWithParam : public ::testing::TestWithParam<double> {};

TEST_P(RandomSelectWithParam, Functional) {
  const double probability = GetParam();
  auto generator = std::mt19937{std::random_device()()};
  auto output =
      ranges::views::generate(beluga::random_select([]() { return 1; }, []() { return 2; }, generator, probability)) |
      ranges::views::take_exactly(1'000'000);
  std::size_t one_count = ranges::count(output, 1);
  std::size_t two_count = ranges::count(output, 2);
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
  for (std::size_t i = 0; i < counters.size(); ++i) {
    ASSERT_NEAR(*(weights.begin() + i), static_cast<double>(counters[i]) / static_cast<double>(range.size()), 0.01);
  }
}

TEST(RandomSample, Functional) {
  const std::size_t samples = 1'000'000;
  auto values = std::array<int, 4>{1, 2, 3, 4};
  auto weights = std::array<double, 4>{0.1, 0.4, 0.3, 0.2};
  auto generator = std::mt19937{std::random_device()()};
  auto output =
      ranges::views::generate(beluga::random_sample(values, weights, generator)) | ranges::views::take_exactly(samples);
  AssertWeights(output, values, weights);
}

class KLDConditionWithParam : public ::testing::TestWithParam<std::tuple<double, std::size_t, std::size_t>> {};

auto GenerateDistinctHashes(std::size_t count) {
  return ranges::views::generate([count, hash = 0UL]() mutable {
    if (hash < count) {
      ++hash;
    }
    return hash;
  });
}

TEST_P(KLDConditionWithParam, Minimum) {
  const std::size_t cluster_count = std::get<1>(GetParam());
  auto output = GenerateDistinctHashes(cluster_count) |
                ranges::views::take_while(beluga::kld_condition(1'000, 0.01, 0.95)) | ranges::to<std::vector>;
  ASSERT_GE(output.size(), 1'000);
}

TEST_P(KLDConditionWithParam, Limit) {
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

template <template <class> class Mixin>
class MockMixin : public ciabatta::mixin<MockMixin<Mixin>, Mixin> {
 public:
  using ciabatta::mixin<MockMixin<Mixin>, Mixin>::mixin;
  MOCK_METHOD(double, generate_random_state, (std::mt19937&));
};

TEST(BaselineGeneration, InstantiateAndCall) {
  auto instance = MockMixin<beluga::BaselineGeneration>();
  EXPECT_CALL(instance, generate_random_state(testing::_)).WillRepeatedly(testing::Return(1.5));
  auto output = instance.generate_samples<std::pair<double, double>>() | ranges::views::take_exactly(50) |
                ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 50);
  for (auto [state, _] : output) {
    ASSERT_EQ(state, 1.5);
  }
}

auto GetParticleSet() {
  return std::vector<std::tuple<int, double, std::size_t>>{
      std::make_tuple(1, 0.3, 0), std::make_tuple(2, 0.1, 0), std::make_tuple(3, 0.4, 0), std::make_tuple(4, 0.2, 0)};
}

TEST(NaiveGeneration, Distribution) {
  auto particles = GetParticleSet();
  auto instance = MockMixin<beluga::NaiveGeneration>();
  auto output = instance.generate_samples_from(particles) | ranges::views::take_exactly(100'000) |
                beluga::views::elements<0> | ranges::to<std::vector>;
  AssertWeights(output, beluga::views::states(particles), beluga::views::weights(particles));
}

TEST(AdaptiveGeneration, Distribution) {
  auto particles = GetParticleSet();
  auto instance = MockMixin<beluga::AdaptiveGeneration>(beluga::AdaptiveGenerationParam{0.001, 0.1});
  auto output = instance.generate_samples_from(particles) | ranges::views::take_exactly(100'000) |
                beluga::views::elements<0> | ranges::to<std::vector>;
  AssertWeights(output, beluga::views::states(particles), beluga::views::weights(particles));
}

class AdaptiveGenerationWithParam : public ::testing::TestWithParam<std::tuple<double, double, double>> {};

TEST_P(AdaptiveGenerationWithParam, RandomParticleCount) {
  const double initial_average_weight = std::get<0>(GetParam());
  const double final_average_weight = std::get<1>(GetParam());
  const double random_particle_probability = std::get<2>(GetParam());

  auto instance = MockMixin<beluga::AdaptiveGeneration>(beluga::AdaptiveGenerationParam{0.001, 0.1});
  EXPECT_CALL(instance, generate_random_state(testing::_)).WillRepeatedly(testing::Return(5));

  auto particles = GetParticleSet();
  auto set_average_weight = [&particles](double value) {
    for (double& weight : beluga::views::weights(particles)) {
      weight = value;
    }
  };

  auto random_particle_percentage = [](const auto& range) {
    return static_cast<double>(ranges::count(range, 5)) / static_cast<double>(range.size());
  };

  auto generate_samples = [&particles, &instance]() {
    return instance.generate_samples_from(particles) | ranges::views::take_exactly(100'000) |
           beluga::views::elements<0> | ranges::to<std::vector>;
  };

  set_average_weight(initial_average_weight);
  auto output = generate_samples();
  ASSERT_EQ(0, random_particle_percentage(output));

  set_average_weight(final_average_weight);
  output = generate_samples();
  ASSERT_NEAR(random_particle_probability, random_particle_percentage(output), 0.01);

  // Once random particles are injected, reset filters
  output = generate_samples();
  ASSERT_EQ(0, random_particle_percentage(output));
}

INSTANTIATE_TEST_SUITE_P(
    AdaptiveParams,
    AdaptiveGenerationWithParam,
    testing::Values(
        std::make_tuple(1.0, 1.5, 0.00),
        std::make_tuple(1.0, 2.0, 0.00),
        std::make_tuple(1.0, 0.5, 0.05),
        std::make_tuple(0.5, 0.1, 0.08),
        std::make_tuple(0.5, 0.0, 0.10)));

TEST(FixedResampling, TakeMaximum) {
  auto instance = MockMixin<beluga::FixedResampling>(beluga::FixedResamplingParam{1'200});
  auto output = ranges::views::generate([]() { return std::make_pair<double, double>(1., 2.); }) |
                instance.take_samples() | ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 1'200);
}

TEST(KldResampling, TakeMaximum) {
  using State = std::pair<double, double>;
  using Particle = std::tuple<State, double, std::size_t>;
  auto instance = MockMixin<beluga::KldResampling>(beluga::KldResamplingParam{0, 1'200, 0.05, 0.05, 3.});
  auto output = ranges::views::generate([]() {
                  return Particle{{1., 1.}, 2., 0};
                }) |
                instance.take_samples() | ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 1'200);
}

TEST(KldResampling, TakeLimit) {
  using State = std::pair<double, double>;
  using Particle = std::tuple<State, double, std::size_t>;
  auto instance = MockMixin<beluga::KldResampling>(beluga::KldResamplingParam{0, 1'200, 0.05, 0.05, 3.});
  auto output = ranges::views::generate([]() {
                  return Particle{{1., 1.}, 2., 0};
                }) |
                ranges::views::intersperse(Particle{{2., 2.}, 2., 0}) |
                ranges::views::intersperse(Particle{{3., 3.}, 2., 0}) | instance.take_samples() |
                ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 135);
}

TEST(KldResampling, TakeMinimum) {
  using State = std::pair<double, double>;
  using Particle = std::tuple<State, double, std::size_t>;
  auto instance = MockMixin<beluga::KldResampling>(beluga::KldResamplingParam{200, 1'200, 0.05, 0.05, 3.});
  auto output = ranges::views::generate([]() {
                  return Particle{{1., 1.}, 2., 0};
                }) |
                ranges::views::intersperse(Particle{{2., 2.}, 2., 0}) |
                ranges::views::intersperse(Particle{{3., 3.}, 2., 0}) | instance.take_samples() |
                ranges::to<std::vector>;
  ASSERT_EQ(output.size(), 200);
}

}  // namespace

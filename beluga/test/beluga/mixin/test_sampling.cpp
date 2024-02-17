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

#include <beluga/mixin/sampling.hpp>
#include <ciabatta/ciabatta.hpp>
#include <range/v3/algorithm/count.hpp>
#include <range/v3/algorithm/fill.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/intersperse.hpp>
#include <range/v3/view/take_exactly.hpp>

namespace {

using testing::_;
using testing::Return;
using testing::ReturnRef;

template <class Input, class Output>
void AssertWeights(Input&& input, Output&& output) {
  std::size_t sample_size = 0UL;
  std::unordered_map<int, std::size_t> buckets;
  for (auto [state, _] : output) {
    ++buckets[state];
    ++sample_size;
  }

  for (auto [state, weight] : input) {
    if (weight > 0.0) {
      ASSERT_NEAR(static_cast<double>(buckets[state]) / static_cast<double>(sample_size), weight, 0.01);
    }
  }
}

template <class Mixin>
using RandomStateGenerator2d = beluga::RandomStateGenerator<Mixin, Sophus::SE2d, Eigen::AlignedBox2d>;

TEST(RandomStateGenerator, InstantiateAndCall) {
  auto generator = std::mt19937{std::random_device()()};
  auto limits = Eigen::AlignedBox2d{Eigen::Vector2d{-1., -1.}, Eigen::Vector2d{1., 1.}};
  auto instance = ciabatta::mixin<RandomStateGenerator2d>{limits};
  for (auto state : instance.generate_samples(generator) | ranges::views::take_exactly(100'000)) {
    EXPECT_TRUE(limits.contains(state.translation()));
  }
}

template <class Mixin>
class MockStorage : public Mixin {
 public:
  using particle_type = std::tuple<int, beluga::Weight>;

  template <class... Args>
  explicit MockStorage(Args&&... rest) : Mixin(std::forward<Args>(rest)...) {}

  MOCK_METHOD(int, make_random_state, (std::mt19937&));
  MOCK_METHOD(const std::vector<particle_type>&, particles, (), (const));
};

TEST(NaiveSampler, Distribution) {
  auto generator = std::mt19937{std::random_device()()};
  auto particles = std::vector<std::tuple<int, beluga::Weight>>{{1, 0.3}, {2, 0.1}, {3, 0.4}, {4, 0.2}};
  auto instance = ciabatta::mixin<MockStorage, beluga::NaiveSampler>{};
  EXPECT_CALL(instance, particles()).WillRepeatedly(ReturnRef(particles));
  auto output = instance.generate_samples_from_particles(generator) | ranges::views::take_exactly(100'000);
  AssertWeights(particles, output);
}

TEST(AdaptiveSampler, Distribution) {
  auto generator = std::mt19937{std::random_device()()};
  auto particles = std::vector<std::tuple<int, beluga::Weight>>{{1, 0.3}, {2, 0.1}, {3, 0.4}, {4, 0.2}};
  auto instance = ciabatta::mixin<MockStorage, beluga::AdaptiveSampler>{beluga::AdaptiveSamplerParam{0.001, 0.1}};
  EXPECT_CALL(instance, particles()).WillRepeatedly(ReturnRef(particles));
  auto output = instance.generate_samples_from_particles(generator) | ranges::views::take_exactly(100'000);
  AssertWeights(particles, output);
}

class AdaptiveSamplerWithParam : public ::testing::TestWithParam<std::tuple<double, double, double>> {};

TEST_P(AdaptiveSamplerWithParam, RandomParticleCount) {
  const auto [initial_average_weight, final_average_weight, random_particle_probability] = GetParam();

  auto instance = ciabatta::mixin<MockStorage, beluga::AdaptiveSampler>{beluga::AdaptiveSamplerParam{0.001, 0.1}};

  // A "random" generated particle will be any particle with value 5
  EXPECT_CALL(instance, make_random_state(_)).WillRepeatedly(Return(5));
  auto random_particle_percentage = [](const auto& range) {
    // Count "random" generated particles by counting the number of 5's
    return static_cast<double>(ranges::count(range, 5, beluga::state)) / static_cast<double>(range.size());
  };

  auto particles = std::vector<std::tuple<int, beluga::Weight>>{{1, 0.0}, {2, 0.0}, {3, 0.0}, {4, 0.0}};
  EXPECT_CALL(instance, particles()).WillRepeatedly(ReturnRef(particles));

  auto create_samples = [&instance]() {
    static auto generator = std::mt19937{std::random_device()()};
    return instance.generate_samples_from_particles(generator) | ranges::views::take_exactly(100'000) |
           ranges::to<std::vector>;
  };

  ranges::fill(beluga::views::weights(particles), initial_average_weight);
  auto output = create_samples();
  ASSERT_EQ(0, random_particle_percentage(output));

  ranges::fill(beluga::views::weights(particles), final_average_weight);
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

TEST(FixedLimiter, TakeMaximum) {
  const std::size_t max = 1200;
  auto instance = ciabatta::mixin<MockStorage, beluga::FixedLimiter>{beluga::FixedLimiterParam{max}};
  auto output = ranges::views::generate([]() { return 1; }) | instance.take_samples();
  ASSERT_EQ(ranges::distance(output), max);
}

template <class Mixin>
struct MockStorageWithCluster : public Mixin {
  using state_type = std::pair<double, double>;
  using particle_type = std::tuple<state_type, beluga::Weight, beluga::Cluster>;

  template <class... Args>
  explicit MockStorageWithCluster(Args&&... rest) : Mixin(std::forward<Args>(rest)...) {}
};

using state = std::pair<double, double>;

TEST(KldLimiter, TakeMaximum) {
  const std::size_t min = 200;
  const std::size_t max = 1200;
  const double k = 3.0;
  const double epsilon = 0.05;
  constexpr std::array kClusteringResolution{1., 1.};
  auto hasher = beluga::spatial_hash<state>{kClusteringResolution};
  auto instance = ciabatta::mixin<MockStorageWithCluster, ciabatta::curry<beluga::KldLimiter, state>::mixin>{
      beluga::KldLimiterParam<state>{min, max, hasher, epsilon, k}};
  auto output = ranges::views::generate([]() { return std::make_pair(1., 1.); }) | instance.take_samples();
  ASSERT_EQ(ranges::distance(output), max);
}

}  // namespace

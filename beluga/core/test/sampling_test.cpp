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

class KLDConditionWithParam : public ::testing::TestWithParam<std::pair<std::size_t, std::size_t>> {};

TEST_P(KLDConditionWithParam, Minimum) {
  const std::size_t cluster_count = GetParam().first;
  const std::size_t fixed_min_samples = 1'000;
  auto predicate = beluga::detail::kld_condition(fixed_min_samples);
  std::size_t cluster = 0;
  for (std::size_t i = 0; i < fixed_min_samples - 1; ++i) {
    ASSERT_TRUE(predicate(cluster)) << "Stopped at " << i + 1 << " samples (Expected: " << fixed_min_samples << ").";
    if (cluster < cluster_count - 1) {
      ++cluster;
    }
  }
}

TEST_P(KLDConditionWithParam, Limit) {
  const std::size_t cluster_count = GetParam().first;
  const std::size_t min_samples = GetParam().second;
  auto predicate = beluga::detail::kld_condition(0, 0.05, 0.95);
  std::size_t cluster = 0;
  for (std::size_t i = 0; i < min_samples - 1; ++i) {
    ASSERT_TRUE(predicate(cluster)) << "Stopped at " << i + 1 << " samples (Expected: " << min_samples << ").";
    if (cluster < cluster_count - 1) {
      ++cluster;
    }
  }
  ASSERT_FALSE(predicate(cluster)) << "Didn't stop at " << min_samples << " samples.";
}

INSTANTIATE_TEST_SUITE_P(
    KLDPairs,
    KLDConditionWithParam,
    testing::Values(
        std::make_pair(3, 8),
        std::make_pair(4, 18),
        std::make_pair(5, 30),
        std::make_pair(6, 44),
        std::make_pair(7, 57),
        std::make_pair(100, 1'713)));

}  // namespace

// Copyright 2026 Ekumen, Inc.
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

#include <algorithm>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <beluga_estimation/common/random.hpp>

namespace {

TEST(RandomNumberGenerator, DefaultConstruction) {
  beluga_estimation::RandomNumberGenerator rng;
  auto value = rng();
  EXPECT_GE(value, beluga_estimation::RandomNumberGenerator::min());
  EXPECT_LE(value, beluga_estimation::RandomNumberGenerator::max());
}

TEST(RandomNumberGenerator, ConstructFromSeed) {
  beluga_estimation::RandomNumberGenerator rng(42);
  auto value = rng();
  EXPECT_GE(value, beluga_estimation::RandomNumberGenerator::min());
  EXPECT_LE(value, beluga_estimation::RandomNumberGenerator::max());
}

TEST(RandomNumberGenerator, SameSeedSameSequence) {
  beluga_estimation::RandomNumberGenerator rng1(42);
  beluga_estimation::RandomNumberGenerator rng2(42);

  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(rng1(), rng2());
  }
}

TEST(RandomNumberGenerator, DifferentSeedsDifferentSequences) {
  beluga_estimation::RandomNumberGenerator rng1(42);
  beluga_estimation::RandomNumberGenerator rng2(43);

  bool all_equal = true;
  for (int i = 0; i < 10; ++i) {
    if (rng1() != rng2()) {
      all_equal = false;
      break;
    }
  }
  EXPECT_FALSE(all_equal);
}

TEST(RandomNumberGenerator, ConstructFromEngine) {
  std::mt19937_64 engine(42);
  beluga_estimation::RandomNumberGenerator rng(engine);
  auto value = rng();
  EXPECT_GE(value, beluga_estimation::RandomNumberGenerator::min());
  EXPECT_LE(value, beluga_estimation::RandomNumberGenerator::max());
}

TEST(RandomNumberGenerator, CopyConstruction) {
  beluga_estimation::RandomNumberGenerator rng1(42);

  rng1();
  rng1();

  beluga_estimation::RandomNumberGenerator rng2(rng1);

  EXPECT_EQ(rng1(), rng2());
  EXPECT_EQ(rng1(), rng2());
  EXPECT_EQ(rng1(), rng2());
}

TEST(RandomNumberGenerator, MoveConstruction) {
  beluga_estimation::RandomNumberGenerator rng1(42);

  rng1();
  rng1();

  beluga_estimation::RandomNumberGenerator rng_copy(rng1);
  auto expected = rng_copy();

  beluga_estimation::RandomNumberGenerator rng2(std::move(rng1));

  EXPECT_EQ(rng2(), expected);
}

TEST(RandomNumberGenerator, CopyAssignment) {
  beluga_estimation::RandomNumberGenerator rng1(42);
  beluga_estimation::RandomNumberGenerator rng2(999);

  rng1();
  rng1();
  rng2();

  rng2 = rng1;

  EXPECT_EQ(rng1(), rng2());
  EXPECT_EQ(rng1(), rng2());
  EXPECT_EQ(rng1(), rng2());
}

TEST(RandomNumberGenerator, MoveAssignment) {
  beluga_estimation::RandomNumberGenerator rng1(42);
  beluga_estimation::RandomNumberGenerator rng2(999);

  rng1();
  rng1();

  beluga_estimation::RandomNumberGenerator rng_copy(rng1);
  auto expected = rng_copy();

  rng2 = std::move(rng1);

  EXPECT_EQ(rng2(), expected);
}

TEST(RandomNumberGenerator, CopyAfterMove) {
  beluga_estimation::RandomNumberGenerator rng1(42);

  beluga_estimation::RandomNumberGenerator rng2(std::move(rng1));

  beluga_estimation::RandomNumberGenerator rng3(rng1);

  EXPECT_THROW({ auto value = rng3(); }, std::runtime_error);
}

TEST(RandomNumberGenerator, MinMaxValues) {
  EXPECT_EQ(beluga_estimation::RandomNumberGenerator::min(), std::numeric_limits<std::uint64_t>::min());
  EXPECT_EQ(beluga_estimation::RandomNumberGenerator::max(), std::numeric_limits<std::uint64_t>::max());
}

TEST(RandomNumberGenerator, GeneratesValuesInRange) {
  beluga_estimation::RandomNumberGenerator rng(42);

  for (int i = 0; i < 100; ++i) {
    auto value = rng();
    EXPECT_GE(value, beluga_estimation::RandomNumberGenerator::min());
    EXPECT_LE(value, beluga_estimation::RandomNumberGenerator::max());
  }
}

TEST(RandomNumberGenerator, WorksWithUniformIntDistribution) {
  beluga_estimation::RandomNumberGenerator rng(42);
  std::uniform_int_distribution<int> dist(1, 6);

  std::vector<int> rolls;
  for (int i = 0; i < 100; ++i) {
    int roll = dist(rng);
    EXPECT_GE(roll, 1);
    EXPECT_LE(roll, 6);
    rolls.push_back(roll);
  }

  EXPECT_GT(*std::max_element(rolls.begin(), rolls.end()), *std::min_element(rolls.begin(), rolls.end()));
}

TEST(RandomNumberGenerator, WorksWithUniformRealDistribution) {
  beluga_estimation::RandomNumberGenerator rng(42);
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  std::vector<double> values;
  for (int i = 0; i < 100; ++i) {
    double value = dist(rng);
    EXPECT_GE(value, 0.0);
    EXPECT_LT(value, 1.0);
    values.push_back(value);
  }

  EXPECT_GT(*std::max_element(values.begin(), values.end()), *std::min_element(values.begin(), values.end()));
}

TEST(RandomNumberGenerator, WorksWithNormalDistribution) {
  beluga_estimation::RandomNumberGenerator rng(42);
  std::normal_distribution<double> dist(0.0, 1.0);

  std::vector<double> values;
  for (int i = 0; i < 1000; ++i) {
    values.push_back(dist(rng));
  }

  double sum = 0.0;
  for (double v : values) {
    sum += v;
  }
  double mean = sum / values.size();

  double variance_sum = 0.0;
  for (double v : values) {
    variance_sum += (v - mean) * (v - mean);
  }
  double stddev = std::sqrt(variance_sum / values.size());

  EXPECT_NEAR(mean, 0.0, 0.1);
  EXPECT_NEAR(stddev, 1.0, 0.1);
}

TEST(RandomNumberGenerator, Reproducibility) {
  beluga_estimation::RandomNumberGenerator rng1(12345);
  beluga_estimation::RandomNumberGenerator rng2(12345);

  std::vector<beluga_estimation::RandomNumberGenerator::result_type> seq1;
  std::vector<beluga_estimation::RandomNumberGenerator::result_type> seq2;

  for (int i = 0; i < 50; ++i) {
    seq1.push_back(rng1());
    seq2.push_back(rng2());
  }

  EXPECT_EQ(seq1, seq2);
}

}  // namespace

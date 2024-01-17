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

#include <gmock/gmock.h>

#include <beluga/algorithm/effective_sample_size.hpp>

namespace {

TEST(EffectiveSampleSize, Empty) {
  auto weights = std::vector<double>{};
  ASSERT_EQ(beluga::effective_sample_size(weights), 0.0);
}

TEST(EffectiveSampleSize, Zeros) {
  auto weights = std::vector{0.0, 0.0, 0.0, 0.0, 0.0};
  ASSERT_EQ(beluga::effective_sample_size(weights), 0.0);
}

TEST(EffectiveSampleSize, Ones) {
  auto weights = std::vector{1.0, 1.0, 1.0, 1.0, 1.0};
  ASSERT_NEAR(beluga::effective_sample_size(weights), 5.0, 0.01);
}

TEST(EffectiveSampleSize, HomogeneousLow) {
  auto weights = std::vector{0.1, 0.1, 0.1, 0.1, 0.1};
  ASSERT_NEAR(beluga::effective_sample_size(weights), 5.0, 0.01);
}

TEST(EffectiveSampleSize, HomogeneousHigh) {
  auto weights = std::vector{100, 100, 100, 100, 100};
  ASSERT_NEAR(beluga::effective_sample_size(weights), 5.0, 0.01);
}

TEST(EffectiveSampleSize, OneZero) {
  auto weights = std::vector{1.0, 0.0};
  ASSERT_NEAR(beluga::effective_sample_size(weights), 1.0, 0.01);
}

TEST(EffectiveSampleSize, OneZeroZero) {
  auto weights = std::vector{1.0, 0.0, 0.0};
  ASSERT_NEAR(beluga::effective_sample_size(weights), 1.0, 0.01);
}

TEST(EffectiveSampleSize, OneOneZero) {
  auto weights = std::vector{1.0, 1.0, 0.0};
  ASSERT_NEAR(beluga::effective_sample_size(weights), 2.0, 0.01);
}

TEST(EffectiveSampleSize, Heterogeneous1) {
  auto weights = std::vector{1.0, 0.5, 0.0};
  ASSERT_NEAR(beluga::effective_sample_size(weights), 1.8, 0.01);
}

TEST(EffectiveSampleSize, Heterogeneous2) {
  auto weights = std::vector{1.0, 0.5, 0.5};
  ASSERT_NEAR(beluga::effective_sample_size(weights), 2.66, 0.01);
}

TEST(EffectiveSampleSize, Particles) {
  struct Particle {
    int state;
    double weight;
  };
  auto particles = std::vector{Particle{1, 1.0}, Particle{1, 0.5}, Particle{1, 0.5}};
  ASSERT_NEAR(beluga::effective_sample_size(particles), 2.66, 0.01);
}

}  // namespace

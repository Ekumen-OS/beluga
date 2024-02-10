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

#include <beluga/random/uniform_grid_distribution.hpp>
#include <beluga/testing/sophus_matchers.hpp>
#include <beluga/testing/sophus_printers.hpp>

#include <beluga/test/static_occupancy_grid.hpp>

#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

namespace {

using beluga::testing::Vector2Near;

TEST(UniformGridDistribution, SingleSlot) {
  constexpr double kTolerance = 0.001;
  constexpr double kResolution = 0.5;
  const auto origin = Sophus::SE2d{Sophus::SO2d{}, Sophus::Vector2d{1.0, 2.0}};
  const auto grid = beluga::testing::StaticOccupancyGrid<1, 1>{{false}, kResolution, origin};
  auto distribution = beluga::UniformGridDistribution{grid};
  auto pose = distribution();
  ASSERT_THAT(pose.translation(), Vector2Near({1.25, 2.25}, kTolerance));
}

TEST(UniformGridDistribution, SingleFreeSlot) {
  constexpr double kTolerance = 0.001;
  constexpr double kResolution = 1.0;
  const auto grid = beluga::testing::StaticOccupancyGrid<5, 5>{
      {true, true, true,  true, true,  //
       true, true, true,  true, true,  //
       true, true, false, true, true,  //
       true, true, true,  true, true,  //
       true, true, true,  true, true},
      kResolution,
  };
  auto distribution = beluga::UniformGridDistribution{grid};
  auto pose = distribution();
  ASSERT_THAT(pose.translation(), Vector2Near({2.5, 2.5}, kTolerance));
}

TEST(UniformGridDistribution, SomeFreeSlots) {
  constexpr std::size_t kSize = 10'000;
  constexpr double kResolution = 1.0;
  const auto grid = beluga::testing::StaticOccupancyGrid<3, 3>{
      {true, false, true,   //
       false, true, false,  //
       true, false, true},
      kResolution,
  };
  auto distribution = beluga::UniformGridDistribution{grid};

  auto output = ranges::views::generate(distribution) | ranges::views::take_exactly(kSize);

  struct bucket_hash {
    std::size_t operator()(const Sophus::Vector2d& s) const noexcept {
      std::size_t h1 = std::hash<double>{}(s.x());
      std::size_t h2 = std::hash<double>{}(s.y());
      return h1 ^ (h2 << 1);
    }
  };

  struct bucket_equal {
    bool operator()(const Sophus::Vector2d& lhs, const Sophus::Vector2d& rhs) const noexcept {
      // good enough, since copies of the same candidate are expected to be identical copies
      return lhs.x() == rhs.x() && lhs.y() == rhs.y();
    }
  };

  std::unordered_map<Sophus::Vector2d, std::size_t, bucket_hash, bucket_equal> buckets;
  for (auto pose : output) {
    ++buckets[pose.translation()];
  }

  constexpr double kTolerance = 0.01;
  ASSERT_EQ(ranges::size(buckets), 4);
  ASSERT_NEAR(static_cast<double>(buckets[Sophus::Vector2d(1.5, 0.5)]) / kSize, 0.25, kTolerance);
  ASSERT_NEAR(static_cast<double>(buckets[Sophus::Vector2d(0.5, 1.5)]) / kSize, 0.25, kTolerance);
  ASSERT_NEAR(static_cast<double>(buckets[Sophus::Vector2d(2.5, 1.5)]) / kSize, 0.25, kTolerance);
  ASSERT_NEAR(static_cast<double>(buckets[Sophus::Vector2d(1.5, 2.5)]) / kSize, 0.25, kTolerance);
}

}  // namespace

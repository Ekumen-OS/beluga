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

#include <optional>

#include <gtest/gtest.h>

#include <beluga_estimation/common/interval.hpp>

namespace {

// --- Interval bound representation ---

TEST(Interval, UntilCreatesOpenUpperBound) {
  auto interval = beluga_estimation::Interval<int>::Until(5);
  ASSERT_FALSE(interval.lower_bound.has_value());
  ASSERT_TRUE(interval.upper_bound.has_value());
  EXPECT_EQ(interval.upper_bound->value, 5);
  EXPECT_FALSE(interval.upper_bound->closed);
}

TEST(Interval, FromCreatesClosedLowerBound) {
  auto interval = beluga_estimation::Interval<int>::From(5);
  ASSERT_TRUE(interval.lower_bound.has_value());
  ASSERT_FALSE(interval.upper_bound.has_value());
  EXPECT_EQ(interval.lower_bound->value, 5);
  EXPECT_TRUE(interval.lower_bound->closed);
}

TEST(Interval, SinceCreatesOpenLowerBound) {
  auto interval = beluga_estimation::Interval<int>::Since(5);
  ASSERT_TRUE(interval.lower_bound.has_value());
  ASSERT_FALSE(interval.upper_bound.has_value());
  EXPECT_EQ(interval.lower_bound->value, 5);
  EXPECT_FALSE(interval.lower_bound->closed);
}

TEST(Interval, DefaultHasNoBounds) {
  beluga_estimation::Interval<int> interval;
  ASSERT_FALSE(interval.lower_bound.has_value());
  ASSERT_FALSE(interval.upper_bound.has_value());
}

// --- Bounded interval factories ---

TEST(Interval, ClosedCreatesClosedBounds) {
  auto interval = beluga_estimation::Interval<int>::Closed(2, 5);
  ASSERT_TRUE(interval.lower_bound.has_value());
  ASSERT_TRUE(interval.upper_bound.has_value());
  EXPECT_EQ(interval.lower_bound->value, 2);
  EXPECT_TRUE(interval.lower_bound->closed);
  EXPECT_EQ(interval.upper_bound->value, 5);
  EXPECT_TRUE(interval.upper_bound->closed);
}

TEST(Interval, OpenCreatesOpenBounds) {
  auto interval = beluga_estimation::Interval<int>::Open(2, 5);
  ASSERT_TRUE(interval.lower_bound.has_value());
  ASSERT_TRUE(interval.upper_bound.has_value());
  EXPECT_EQ(interval.lower_bound->value, 2);
  EXPECT_FALSE(interval.lower_bound->closed);
  EXPECT_EQ(interval.upper_bound->value, 5);
  EXPECT_FALSE(interval.upper_bound->closed);
}

TEST(Interval, BetweenIsAliasOfOpen) {
  auto interval = beluga_estimation::Interval<int>::Between(2, 5);
  ASSERT_TRUE(interval.lower_bound.has_value());
  ASSERT_TRUE(interval.upper_bound.has_value());
  EXPECT_EQ(interval.lower_bound->value, 2);
  EXPECT_FALSE(interval.lower_bound->closed);
  EXPECT_EQ(interval.upper_bound->value, 5);
  EXPECT_FALSE(interval.upper_bound->closed);
}

TEST(Interval, RightOpenCreatesHalfOpenInterval) {
  // [2, 5)
  {
    auto interval = beluga_estimation::Interval<int>::RightOpen(2, 5);
    ASSERT_TRUE(interval.lower_bound.has_value());
    ASSERT_TRUE(interval.upper_bound.has_value());
    EXPECT_EQ(interval.lower_bound->value, 2);
    EXPECT_TRUE(interval.lower_bound->closed);
    EXPECT_EQ(interval.upper_bound->value, 5);
    EXPECT_FALSE(interval.upper_bound->closed);
  }
  // [2, ∞) — upper bound is optional.
  {
    auto interval = beluga_estimation::Interval<int>::RightOpen(2);
    ASSERT_TRUE(interval.lower_bound.has_value());
    ASSERT_FALSE(interval.upper_bound.has_value());
    EXPECT_EQ(interval.lower_bound->value, 2);
    EXPECT_TRUE(interval.lower_bound->closed);
  }
}

TEST(Interval, LeftOpenCreatesHalfOpenInterval) {
  // (2, 5]
  {
    auto interval = beluga_estimation::Interval<int>::LeftOpen(2, 5);
    ASSERT_TRUE(interval.lower_bound.has_value());
    ASSERT_TRUE(interval.upper_bound.has_value());
    EXPECT_EQ(interval.lower_bound->value, 2);
    EXPECT_FALSE(interval.lower_bound->closed);
    EXPECT_EQ(interval.upper_bound->value, 5);
    EXPECT_TRUE(interval.upper_bound->closed);
  }
  // (-∞, 5] — lower bound is optional.
  {
    auto interval = beluga_estimation::Interval<int>::LeftOpen(std::nullopt, 5);
    ASSERT_FALSE(interval.lower_bound.has_value());
    ASSERT_TRUE(interval.upper_bound.has_value());
    EXPECT_EQ(interval.upper_bound->value, 5);
    EXPECT_TRUE(interval.upper_bound->closed);
  }
}

}  // namespace

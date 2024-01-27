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

#include <beluga/actions/assign.hpp>
#include <beluga/actions/normalize.hpp>

#include <range/v3/algorithm/equal.hpp>

namespace {

TEST(NormalizeAction, DefaultExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(4.0))};
  input |= beluga::actions::normalize(2.0);
  ASSERT_EQ(input.front(), std::make_tuple(5, 2.0));
}

TEST(NormalizeAction, SequencedExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(4.0))};
  input |= beluga::actions::normalize(std::execution::seq, 2.0);
  ASSERT_EQ(input.front(), std::make_tuple(5, 2.0));
}

TEST(NormalizeAction, ParallelExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(4.0))};
  input |= beluga::actions::normalize(std::execution::par, 2.0);
  ASSERT_EQ(input.front(), std::make_tuple(5, 2.0));
}

TEST(NormalizeAction, DefaultFactor) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(4.0))};
  input |= beluga::actions::normalize(std::execution::seq);
  ASSERT_EQ(input.front(), std::make_tuple(5, 1.0));
}

TEST(NormalizeAction, DefaultFactorAndExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(4.0))};
  input |= beluga::actions::normalize;
  ASSERT_EQ(input.front(), std::make_tuple(5, 1.0));
}

TEST(NormalizeAction, EmptyInputRange) {
  auto input = std::vector<std::tuple<int, beluga::Weight>>{};
  input |= beluga::actions::normalize(2.0);
  ASSERT_TRUE(input.empty());
}

TEST(NormalizeAction, MultipleParticles) {
  auto input = std::vector{
      std::make_tuple(5, beluga::Weight(4.0)),  //
      std::make_tuple(8, beluga::Weight(2.0)),  //
      std::make_tuple(3, beluga::Weight(6.0))};
  input |= beluga::actions::normalize(2.0);
  ASSERT_EQ(input.size(), 3);
  ASSERT_EQ(input[0], std::make_tuple(5, 2.0));
  ASSERT_EQ(input[1], std::make_tuple(8, 1.0));
  ASSERT_EQ(input[2], std::make_tuple(3, 3.0));
}

TEST(NormalizeAction, MultipleElements) {
  auto input = std::vector{4.0, 2.0, 6.0};
  input |= beluga::actions::normalize(2.0);
  ASSERT_EQ(input.size(), 3);
  ASSERT_EQ(input[0], 2.0);
  ASSERT_EQ(input[1], 1.0);
  ASSERT_EQ(input[2], 3.0);
}

TEST(NormalizeAction, ZeroFactor) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(4.0))};
  input |= beluga::actions::normalize(0.0);
  ASSERT_EQ(input.front(), std::make_tuple(5, beluga::Weight(4.0)));  // No change
}

TEST(NormalizeAction, NegativeFactor) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(4.0))};
  input |= beluga::actions::normalize(-2.0);
  ASSERT_EQ(input.front(), std::make_tuple(5, beluga::Weight(4.0)));  // No change
}

}  // namespace

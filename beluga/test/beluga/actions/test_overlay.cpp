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
#include <gtest/gtest.h>

#include <cmath>
#include <execution>
#include <string>
#include <tuple>
#include <vector>

#include "beluga/actions/overlay.hpp"
#include "beluga/primitives.hpp"

namespace {

TEST(OverlayAction, DefaultExecutionPolicyAndBoolInput) {
  auto input = std::vector{false, false, false, false, false, false, false, false, false};
  const auto mask = std::vector{false, false, false, true, false, true, false, false, false};
  const bool mask_value = true;

  input |= beluga::actions::overlay(mask, mask_value);

  ASSERT_THAT(input, ::testing::Pointwise(::testing::Eq(), mask));
}

TEST(OverlayAction, DefaultExecutionPolicyAndIntInput) {
  auto input = std::vector{1, 2, 3, 4, 5, 6, 7, 8, 9};
  const auto mask = std::vector{false, false, false, true, false, true, false, false, false};
  const int mask_value = 5;

  input |= beluga::actions::overlay(mask, mask_value);

  const auto expected_output = std::vector{1, 2, 3, 5, 5, 5, 7, 8, 9};

  ASSERT_THAT(input, ::testing::Pointwise(::testing::Eq(), expected_output));
}

TEST(OverlayAction, DefaultExecutionPolicyAndStringInput) {
  auto input = std::vector<std::string>{"b", "e", "l", "u", "g", "a", "c", "p", "p"};
  const auto mask = std::vector{false, false, false, false, false, false, false, true, true};
  const std::string mask_value = "+";

  input |= beluga::actions::overlay(mask, mask_value);

  const auto expected_output = std::vector{"b", "e", "l", "u", "g", "a", "c", "+", "+"};

  ASSERT_THAT(input, ::testing::Pointwise(::testing::Eq(), expected_output));
}

TEST(OverlayAction, DefaultExecutionPolicyAndDoubleInput) {
  auto input = std::vector{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9};
  const auto mask = std::vector<bool>{false, false, false, true, false, true, false, false, false};
  const double mask_value = 5.5;

  input |= beluga::actions::overlay(mask, mask_value);

  const auto expected_output = std::vector{1.1, 2.2, 3.3, 5.5, 5.5, 5.5, 7.7, 8.8, 9.9};

  ASSERT_THAT(input, ::testing::Pointwise(::testing::Eq(), expected_output));
}

TEST(OverlayAction, DefaultExecutionPolicyAndAllowConvertible) {
  auto input = std::vector{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9};
  const auto mask = std::vector<bool>{false, false, false, true, false, true, false, false, false};
  const int mask_value = 5;

  input |= beluga::actions::overlay(mask, mask_value);

  const auto expected_output = std::vector{1.1, 2.2, 3.3, 5.0, 5.5, 5.0, 7.7, 8.8, 9.9};

  ASSERT_THAT(input, ::testing::Pointwise(::testing::Eq(), expected_output));
}

TEST(OverlayAction, DefaultExecutionPolicyWithoutClosure) {
  auto input = std::vector{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9};
  const auto mask = std::vector{false, false, false, true, false, true, false, false, false};
  const double mask_value = 5.5;

  input = beluga::actions::overlay(input, mask, mask_value);

  const auto expected_output = std::vector{1.1, 2.2, 3.3, 5.5, 5.5, 5.5, 7.7, 8.8, 9.9};

  ASSERT_THAT(input, ::testing::Pointwise(::testing::Eq(), expected_output));
}

TEST(OverlayAction, SeqExecutionPolicyWithoutClosure) {
  auto input = std::vector{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9};
  const auto mask = std::vector{false, false, false, true, false, true, false, false, false};
  const double mask_value = 5.5;

  input = beluga::actions::overlay(std::execution::seq, input, mask, mask_value);

  const auto expected_output = std::vector{1.1, 2.2, 3.3, 5.5, 5.5, 5.5, 7.7, 8.8, 9.9};

  ASSERT_THAT(input, ::testing::Pointwise(::testing::Eq(), expected_output));
}

TEST(OverlayAction, SeqExecutionPolicy) {
  auto input = std::vector{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9};
  const auto mask = std::vector{false, false, false, true, false, true, false, false, false};
  const double mask_value = 5.5;

  input |= beluga::actions::overlay(std::execution::seq, mask, mask_value);

  const auto expected_output = std::vector{1.1, 2.2, 3.3, 5.5, 5.5, 5.5, 7.7, 8.8, 9.9};

  ASSERT_THAT(input, ::testing::Pointwise(::testing::Eq(), expected_output));
}

TEST(OverlayAction, ParallelExecutionPolicy) {
  auto input = std::vector{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9};
  const auto mask = std::vector{false, false, false, true, false, true, false, false, false};
  const double mask_value = 5.5;

  input |= beluga::actions::overlay(std::execution::par, mask, mask_value);

  const auto expected_output = std::vector{1.1, 2.2, 3.3, 5.5, 5.5, 5.5, 7.7, 8.8, 9.9};

  ASSERT_THAT(input, ::testing::Pointwise(::testing::Eq(), expected_output));
}

TEST(OverlayAction, Temporary) {
  auto input = std::vector{1, 2, 3, 4, 5, 6, 7, 8, 9};
  auto action = beluga::actions::overlay(std::vector{false, false, false, true, false, true, false, false, false}, 5);

  input |= action;

  const auto expected_output = std::vector{1, 2, 3, 5, 5, 5, 7, 8, 9};

  ASSERT_THAT(input, ::testing::Pointwise(::testing::Eq(), expected_output));
}

}  // namespace

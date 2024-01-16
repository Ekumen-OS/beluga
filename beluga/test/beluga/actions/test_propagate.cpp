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

#include <gtest/gtest.h>

#include <beluga/actions/assign.hpp>
#include <beluga/actions/propagate.hpp>
#include <beluga/views/sample.hpp>

#include <range/v3/algorithm/equal.hpp>
#include <range/v3/view/take_exactly.hpp>

namespace {

TEST(PropagateAction, DefaultExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(1.0))};
  input |= beluga::actions::propagate([](int value) { return ++value; });
  ASSERT_EQ(input.front(), std::make_tuple(6, 1.0));
}

TEST(PropagateAction, SequencedExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(1.0))};
  input |= beluga::actions::propagate(std::execution::seq, [](int value) { return ++value; });
  ASSERT_EQ(input.front(), std::make_tuple(6, 1.0));
}

TEST(PropagateAction, ParallelExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(1.0))};
  input |= beluga::actions::propagate(std::execution::par, [](int value) { return ++value; });
  ASSERT_EQ(input.front(), std::make_tuple(6, 1.0));
}

TEST(PropagateAction, Composition) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(1.0))};
  input |= beluga::actions::propagate([](int value) { return --value; }) |  //
           beluga::views::sample |                                          //
           ranges::views::take_exactly(5) |                                 //
           beluga::actions::assign;
  ASSERT_TRUE(ranges::equal(beluga::views::states(input), std::vector{4, 4, 4, 4, 4}));
}

TEST(PropagateAction, StatefulModel) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(1.0))};
  auto model = [value = 0](int) mutable { return value++; };
  input |= beluga::views::sample |           //
           ranges::views::take_exactly(5) |  //
           beluga::actions::assign |         //
           beluga::actions::propagate(std::ref(model));
  ASSERT_TRUE(ranges::equal(beluga::views::states(input), std::vector{0, 1, 2, 3, 4}));
  ASSERT_EQ(model(0), 5);  // the model was passed by reference
}

}  // namespace

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
#include <beluga/actions/reweight.hpp>

#include <range/v3/algorithm/equal.hpp>
#include <range/v3/view/intersperse.hpp>

namespace {

TEST(ReweightAction, DefaultExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(2.0))};
  input |= beluga::actions::reweight([](int value) { return value; });
  ASSERT_EQ(input.front(), std::make_tuple(5, 10.0));
}

TEST(ReweightAction, SequencedExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(2.0))};
  input |= beluga::actions::reweight(std::execution::seq, [](int value) { return value; });
  ASSERT_EQ(input.front(), std::make_tuple(5, 10.0));
}

TEST(ReweightAction, ParallelExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(2.0))};
  input |= beluga::actions::reweight(std::execution::par, [](int value) { return value; });
  ASSERT_EQ(input.front(), std::make_tuple(5, 10.0));
}

TEST(ReweightAction, Composition) {
  auto input = std::vector{std::make_tuple(4, beluga::Weight(0.5)), std::make_tuple(4, beluga::Weight(1.0))};
  input |= beluga::actions::reweight([](int value) { return value; }) |           //
           ranges::views::intersperse(std::make_tuple(5, beluga::Weight(1.0))) |  //
           beluga::actions::assign;
  auto weights = input | beluga::views::weights | ranges::to<std::vector>;
  ASSERT_THAT(weights, testing::ElementsAre(2, 1, 4));
}

TEST(ReweightAction, StatefulModel) {
  auto input = std::vector{std::make_tuple(4, beluga::Weight(0.5)), std::make_tuple(4, beluga::Weight(1.0))};
  auto model = [value = 0](int) mutable { return value++; };
  input |= ranges::views::intersperse(std::make_tuple(5, beluga::Weight(1.0))) |  //
           beluga::actions::assign |                                              //
           beluga::actions::reweight(std::ref(model));
  auto weights = input | beluga::views::weights | ranges::to<std::vector>;
  ASSERT_THAT(weights, testing::ElementsAre(0, 1, 2));
  ASSERT_EQ(model(0), 3);  // the model was passed by reference
}

}  // namespace

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

#include <list>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <beluga/actions/assign.hpp>

#include <range/v3/action/drop.hpp>
#include <range/v3/action/remove.hpp>
#include <range/v3/algorithm/equal.hpp>
#include <range/v3/view/move.hpp>
#include <range/v3/view/reverse.hpp>
#include <range/v3/view/transform.hpp>

namespace {

TEST(AssignAction, ViewToAction) {
  auto input = std::vector{1, 2, 3};
  input |= ranges::views::reverse | beluga::actions::assign;
  ASSERT_TRUE(ranges::equal(input, std::vector{3, 2, 1}));
}

TEST(AssignAction, MoveOnlyRange) {
  auto input = std::vector<std::unique_ptr<int>>{};
  input.emplace_back(std::make_unique<int>(1));
  input.emplace_back(std::make_unique<int>(2));
  input.emplace_back(std::make_unique<int>(3));
  input |= ranges::views::reverse | ranges::views::move | beluga::actions::assign;
  ASSERT_EQ(*input.front(), 3);
  ASSERT_EQ(*input.back(), 1);
}

TEST(AssignAction, ViewToActionComposition) {
  auto reverse_and_assign = ranges::views::reverse | beluga::actions::assign;
  auto input = std::vector{1, 2, 3};
  input |= reverse_and_assign;
  ASSERT_TRUE(ranges::equal(input, std::vector{3, 2, 1}));
}

TEST(AssignAction, ViewToActionCall) {
  auto input = std::vector{1, 2, 3};
  beluga::actions::assign(input, ranges::views::reverse);
  ASSERT_TRUE(ranges::equal(input, std::vector{3, 2, 1}));
}

TEST(AssignAction, ActionComposition) {
  auto input = std::vector{1, 2, 3};
  input |= ranges::actions::drop(1) | beluga::actions::assign;
  ASSERT_TRUE(ranges::equal(input, std::vector{2, 3}));
}

TEST(AssignAction, ActionViewComposition) {
  auto input = std::vector{1, 2, 3};
  input |= ranges::actions::drop(1) | ranges::views::reverse | beluga::actions::assign;
  ASSERT_TRUE(ranges::equal(input, std::vector{3, 2}));
}

TEST(AssignAction, ActionViewActionComposition) {
  auto input = std::vector{1, 2, 3};
  input |= ranges::actions::drop(1) |                                      //
           ranges::views::reverse |                                        //
           beluga::actions::assign |                                       //
           ranges::actions::drop(1) |                                      //
           ranges::views::transform([](auto value) { return ++value; }) |  //
           beluga::actions::assign;
  ASSERT_TRUE(ranges::equal(input, std::vector{3}));
}

TEST(AssignAction, List) {
  auto input = std::list{1, 2, 3};
  input |= ranges::actions::remove(2) | ranges::views::reverse | beluga::actions::assign;
  ASSERT_TRUE(ranges::equal(input, std::list{3, 1}));
}

}  // namespace

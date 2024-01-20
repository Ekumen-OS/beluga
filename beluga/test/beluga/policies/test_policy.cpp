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

#include <beluga/policies/policy.hpp>

namespace {

using testing::Return;

struct MockPredicate {
  MOCK_METHOD(bool, eval, ());
  bool operator()() { return eval(); }
  auto policy() { return beluga::make_policy_closure(std::ref(*this)); }
};

TEST(Policy, AndTrue) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() && condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(true));
  EXPECT_CALL(condition2, eval()).WillOnce(Return(true));
  ASSERT_EQ(combined(), true);
}

TEST(Policy, AndFalseShortCircuit) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() && condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(false));
  EXPECT_CALL(condition2, eval()).Times(0);
  ASSERT_EQ(combined(), false);
}

TEST(Policy, AndFalse) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() & condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(false));
  EXPECT_CALL(condition2, eval()).WillOnce(Return(true));
  ASSERT_EQ(combined(), false);
}

TEST(Policy, OrFalse) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() || condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(false));
  EXPECT_CALL(condition2, eval()).WillOnce(Return(false));
  ASSERT_EQ(combined(), false);
}

TEST(Policy, OrTrueShortCircuit) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() || condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(true));
  EXPECT_CALL(condition2, eval()).Times(0);
  ASSERT_EQ(combined(), true);
}

TEST(Policy, OrTrue) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() | condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(true));
  EXPECT_CALL(condition2, eval()).WillOnce(Return(false));
  ASSERT_EQ(combined(), true);
}

TEST(Policy, Not) {
  MockPredicate condition;
  auto negated = !condition.policy();
  EXPECT_CALL(condition, eval()).WillOnce(Return(false));
  ASSERT_EQ(negated(), true);
  EXPECT_CALL(condition, eval()).WillOnce(Return(true));
  ASSERT_EQ(negated(), false);
}

TEST(Policy, SameArgumentCombinations) {
  auto is_even = beluga::make_policy_closure([](int n) { return n % 2 == 0; });
  auto never = beluga::make_policy_closure([](int) { return false; });
  auto policy1 = is_even || never;
  ASSERT_EQ(policy1(2), true);
  ASSERT_EQ(policy1(3), false);
  auto policy2 = never || is_even;
  ASSERT_EQ(policy2(2), true);
  ASSERT_EQ(policy2(3), false);
}

TEST(Policy, DifferentArgumentCombinations) {
  auto is_even = beluga::make_policy_closure([](int n) { return n % 2 == 0; });
  auto always = beluga::make_policy_closure([]() { return true; });
  auto policy1 = is_even && always;
  ASSERT_EQ(policy1(2), true);
  ASSERT_EQ(policy1(3), false);
  auto policy2 = always && is_even;
  ASSERT_EQ(policy2(2), true);
  ASSERT_EQ(policy2(3), false);
}

}  // namespace

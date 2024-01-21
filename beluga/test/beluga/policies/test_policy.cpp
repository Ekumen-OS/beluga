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
  ASSERT_TRUE(combined());
}

TEST(Policy, AndFalseShortCircuit) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() && condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(false));
  EXPECT_CALL(condition2, eval()).Times(0);
  ASSERT_FALSE(combined());
}

TEST(Policy, AndFalse) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() & condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(false));
  EXPECT_CALL(condition2, eval()).WillOnce(Return(true));
  ASSERT_FALSE(combined());
}

TEST(Policy, OrFalse) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() || condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(false));
  EXPECT_CALL(condition2, eval()).WillOnce(Return(false));
  ASSERT_FALSE(combined());
}

TEST(Policy, OrTrueShortCircuit) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() || condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(true));
  EXPECT_CALL(condition2, eval()).Times(0);
  ASSERT_TRUE(combined());
}

TEST(Policy, OrTrue) {
  MockPredicate condition1;
  MockPredicate condition2;
  auto combined = condition1.policy() | condition2.policy();
  EXPECT_CALL(condition1, eval()).WillOnce(Return(true));
  EXPECT_CALL(condition2, eval()).WillOnce(Return(false));
  ASSERT_TRUE(combined());
}

TEST(Policy, Not) {
  MockPredicate condition;
  auto negated = !condition.policy();
  EXPECT_CALL(condition, eval()).WillOnce(Return(false));
  ASSERT_TRUE(negated());
  EXPECT_CALL(condition, eval()).WillOnce(Return(true));
  ASSERT_FALSE(negated());
}

TEST(Policy, SameArgumentCombinations) {
  auto is_even = beluga::make_policy_closure([](int n) { return n % 2 == 0; });
  auto never = beluga::make_policy_closure([](int) { return false; });
  auto policy1 = is_even || never;
  ASSERT_TRUE(policy1(2));
  ASSERT_FALSE(policy1(3));
  auto policy2 = never || is_even;
  ASSERT_TRUE(policy2(2));
  ASSERT_FALSE(policy2(3));
}

TEST(Policy, DifferentArgumentCombinations) {
  auto is_even = beluga::make_policy_closure([](int n) { return n % 2 == 0; });
  auto always = beluga::make_policy_closure([]() { return true; });
  auto policy1 = is_even && always;
  ASSERT_TRUE(policy1(2));
  ASSERT_FALSE(policy1(3));
  auto policy2 = always && is_even;
  ASSERT_TRUE(policy2(2));
  ASSERT_FALSE(policy2(3));
}

TEST(Policy, AnyAssignment) {
  beluga::any_policy<double> policy;
  policy = beluga::make_policy_closure([](double value) { return value > 0.0; });
  ASSERT_TRUE(policy(1.0));
  ASSERT_FALSE(policy(-1.0));
  policy = beluga::make_policy_closure([]() { return true; });
  ASSERT_TRUE(policy(1.0));
  ASSERT_TRUE(policy(-1.0));
}

TEST(Policy, AnyComposition) {
  beluga::any_policy<double> policy;
  policy = beluga::make_policy_closure([](double value) { return value > 0.0; });
  ASSERT_TRUE(policy(1.0));
  ASSERT_FALSE(policy(-1.0));
  policy = policy && beluga::make_policy_closure([]() { return true; });
  ASSERT_TRUE(policy(1.0));
  ASSERT_FALSE(policy(-1.0));
}

}  // namespace

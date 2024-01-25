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

#include <beluga/policies/every_n.hpp>

namespace {

TEST(EveryNPolicy, TriggerOnNthCall) {
  auto policy = beluga::policies::every_n(3);  // Trigger every 3rd call
  ASSERT_FALSE(policy());
  ASSERT_FALSE(policy());
  ASSERT_TRUE(policy());
}

TEST(EveryNPolicy, NoTriggerBeforeN) {
  auto policy = beluga::policies::every_n(4);  // Trigger every 4th call
  ASSERT_FALSE(policy());
  ASSERT_FALSE(policy());
  ASSERT_FALSE(policy());
}

TEST(EveryNPolicy, TriggerOnMultipleN) {
  auto policy = beluga::policies::every_n(2);  // Trigger every 2nd call
  ASSERT_FALSE(policy());
  ASSERT_TRUE(policy());
  ASSERT_FALSE(policy());
  ASSERT_TRUE(policy());
}

}  // namespace

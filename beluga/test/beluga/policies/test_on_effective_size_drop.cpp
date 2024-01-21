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

#include <beluga/policies/on_effective_size_drop.hpp>

namespace {

TEST(OnEffectiveSizeDropPolicy, TriggerOnDropBelowHalf) {
  auto policy = beluga::policies::on_effective_size_drop;
  auto weights1 = std::vector{1.0, 1.0, 1.0, 1.0, 1.0};
  auto weights2 = std::vector{1.0, 1.0, 0.0, 0.0, 0.0};  // Fewer effective particles

  ASSERT_FALSE(policy(weights1));  // Initial ESS should not trigger the policy
  ASSERT_TRUE(policy(weights2));   // ESS drops below half on the second call, triggering the policy
}

TEST(OnEffectiveSizeDropPolicy, NoTriggerAboveHalf) {
  auto policy = beluga::policies::on_effective_size_drop;
  auto weights1 = std::vector{1.0, 1.0, 1.0, 1.0, 1.0};
  auto weights2 = std::vector{1.0, 1.0, 1.0, 0.0, 0.0};  // Still above half

  ASSERT_FALSE(policy(weights1));  // Initial ESS should not trigger the policy
  ASSERT_FALSE(policy(weights1));  // ESS remains above half, no trigger
}

}  // namespace

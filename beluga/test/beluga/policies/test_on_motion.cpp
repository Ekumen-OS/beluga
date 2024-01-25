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

#include <beluga/policies/on_motion.hpp>

namespace {

TEST(OnMotionPolicy, TriggerOnMotion) {
  auto policy = beluga::policies::on_motion(0.1, 0.05);
  Sophus::SE2d pose1(Sophus::SO2d(0.2), Eigen::Vector2d(1.0, 2.0));
  Sophus::SE2d pose2(Sophus::SO2d(0.25), Eigen::Vector2d(1.2, 2.2));

  ASSERT_TRUE(policy(pose1));   // First pose triggers the policy
  ASSERT_FALSE(policy(pose1));  // Same pose should not trigger again
  ASSERT_TRUE(policy(pose2));   // Second pose triggers the policy
}

TEST(OnMotionPolicy, NoTriggerWithoutMotion) {
  auto policy = beluga::policies::on_motion(0.1, 0.05);
  Sophus::SE2d pose1(Sophus::SO2d(0.1), Eigen::Vector2d(1.0, 2.0));
  Sophus::SE2d pose2(Sophus::SO2d(0.1), Eigen::Vector2d(1.05, 2.05));

  ASSERT_TRUE(policy(pose1));   // First pose triggers the policy
  ASSERT_FALSE(policy(pose2));  // Small motion should not trigger the policy
}

}  // namespace

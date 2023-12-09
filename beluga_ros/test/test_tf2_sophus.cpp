// Copyright 2023 Ekumen, Inc.
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
#include <beluga_ros/tf2_sophus.hpp>

namespace {

using SophusTestCases = testing::Types<Sophus::SE2d, Sophus::SE2f, Sophus::SE3d, Sophus::SE3f>;

template <typename T>
class TransformConvertTest : public testing::Test {
 public:
  using TypeParam = T;
};

TYPED_TEST_SUITE(TransformConvertTest, SophusTestCases, );

TYPED_TEST(TransformConvertTest, ThereAndBackAgain) {
  auto message1 = beluga_ros::msg::Transform{};
  auto message2 = beluga_ros::msg::Transform{};
  message1.translation.x = 1;
  message1.translation.y = 2;
  message1.translation.z = 0;
  message1.rotation.x = 0;
  message1.rotation.y = 0;
  message1.rotation.z = 0;
  message1.rotation.w = 1;
  auto instance = typename TestFixture::TypeParam{};
  tf2::convert(message1, instance);
  tf2::convert(instance, message2);
  ASSERT_EQ(message1, message2);
}

template <typename T>
class PoseConvertTest : public testing::Test {
 public:
  using TypeParam = T;
};

TYPED_TEST_SUITE(PoseConvertTest, SophusTestCases, );

TYPED_TEST(PoseConvertTest, ThereAndBackAgain) {
  auto message1 = beluga_ros::msg::Pose{};
  auto message2 = beluga_ros::msg::Pose{};
  message1.position.x = 1;
  message1.position.y = 2;
  message1.position.z = 0;
  message1.orientation.x = 0;
  message1.orientation.y = 0;
  message1.orientation.z = 0;
  message1.orientation.w = 1;
  auto instance = typename TestFixture::TypeParam{};
  tf2::fromMsg(message1, instance);
  tf2::toMsg(instance, message2);
  ASSERT_EQ(message1, message2);
}

}  // namespace

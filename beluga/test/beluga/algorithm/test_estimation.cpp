// Copyright 2022 Ekumen, Inc.
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

#include <beluga/algorithm/estimation.hpp>
#include <ciabatta/ciabatta.hpp>
#include <range/v3/view/all.hpp>

#include "../../utils/sophus_matchers.hpp"

namespace {

using testing::ReturnRef;
using testing::SO2Eq;
using testing::Vector2Eq;

using Constants = Sophus::Constants<double>;
using Eigen::Vector2d;
using Sophus::SE2d;
using Sophus::SO2d;

template <template <class> class Mixin>
class MockMixin : public ciabatta::mixin<MockMixin<Mixin>, Mixin> {
 public:
  using ciabatta::mixin<MockMixin<Mixin>, Mixin>::mixin;
  MOCK_METHOD(const std::vector<Sophus::SE2d>&, states, (), (const));
};

TEST(SimpleEstimation, Position) {
  const auto mixin = MockMixin<beluga::SimpleEstimation>{};
  const auto states = std::vector{SE2d{SO2d{0.0}, Vector2d{1.0, 2.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}};
  EXPECT_CALL(mixin, states()).WillOnce(ReturnRef(states));
  const auto [pose, covariance] = mixin.estimated_pose();
  ASSERT_THAT(pose.translation(), Vector2Eq(Vector2d{0.5, 1.0}));
  ASSERT_NEAR(covariance(0, 0), 0.5, 0.001);
  ASSERT_NEAR(covariance(0, 1), 1.0, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.0, 0.001);
  ASSERT_NEAR(covariance(1, 0), 1.0, 0.001);
  ASSERT_NEAR(covariance(1, 1), 2.0, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.0, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.0, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.0, 0.001);
  ASSERT_NEAR(covariance(2, 2), 0.0, 0.001);
}

TEST(SimpleEstimation, Rotation) {
  const auto mixin = MockMixin<beluga::SimpleEstimation>{};
  const auto states =
      std::vector{SE2d{SO2d{-Constants::pi() / 2}, Vector2d{0.0, 0.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}};
  EXPECT_CALL(mixin, states()).WillOnce(ReturnRef(states));
  const auto [pose, covariance] = mixin.estimated_pose();
  ASSERT_THAT(pose.so2(), SO2Eq(SO2d{-Constants::pi() / 4}));
  ASSERT_NEAR(covariance(0, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(0, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 2), 0.693, 0.001);
}

TEST(SimpleEstimation, ThreePoints) {
  const auto mixin = MockMixin<beluga::SimpleEstimation>{};
  const auto states = std::vector{
      SE2d{SO2d{Constants::pi() / 6}, Vector2d{0.0, -3.0}}, SE2d{SO2d{Constants::pi() / 2}, Vector2d{1.0, -2.0}},
      SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, -1.0}}, SE2d{SO2d{0.0}, Vector2d{3.0, 0.0}}};
  EXPECT_CALL(mixin, states()).WillOnce(ReturnRef(states));
  const auto [pose, covariance] = mixin.estimated_pose();
  ASSERT_THAT(pose.so2(), SO2Eq(SO2d{Constants::pi() / 4}));
  ASSERT_NEAR(covariance(0, 0), 1.666, 0.001);
  ASSERT_NEAR(covariance(0, 1), 1.666, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 0), 1.666, 0.001);
  ASSERT_NEAR(covariance(1, 1), 1.666, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 2), 0.357, 0.001);
}

TEST(SimpleEstimation, CanceledOutAngles) {
  const auto mixin = MockMixin<beluga::SimpleEstimation>{};
  const auto states = std::vector{
      SE2d{SO2d{Constants::pi() / 2}, Vector2d{0.0, 0.0}}, SE2d{SO2d{-Constants::pi() / 2}, Vector2d{0.0, 0.0}}};
  EXPECT_CALL(mixin, states()).WillOnce(ReturnRef(states));
  const auto [pose, covariance] = mixin.estimated_pose();
  ASSERT_THAT(pose.so2(), SO2Eq(SO2d{0.0}));
  ASSERT_NEAR(covariance(0, 0), 0.0, 0.001);
  ASSERT_NEAR(covariance(0, 1), 0.0, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.0, 0.001);
  ASSERT_NEAR(covariance(1, 0), 0.0, 0.001);
  ASSERT_NEAR(covariance(1, 1), 0.0, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.0, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.0, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.0, 0.001);
  ASSERT_EQ(covariance(2, 2), std::numeric_limits<double>::infinity());
}

}  // namespace

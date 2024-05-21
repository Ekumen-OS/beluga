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
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

#include "beluga/testing/sophus_matchers.hpp"

namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Sophus::SE2d;
using Sophus::SO2d;

using beluga::testing::SE2Near;
using beluga::testing::SO2Near;
using beluga::testing::Vector2Near;
using beluga::testing::Vector3Near;

using testing::Not;

TEST(SophusMatchers, Vector2Near) {
  constexpr double kHighTolerance = 0.6;
  constexpr double kLowTolerance = 0.4;
  ASSERT_THAT(Vector2d(1.0, 2.0), Vector2Near({1.5, 1.5}, kHighTolerance));
  ASSERT_THAT(Vector2d(1.0, 2.0), Not(Vector2Near({1.5, 1.5}, kLowTolerance)));
}

TEST(SophusMatchers, Vector3Near) {
  constexpr double kHighTolerance = 0.6;
  constexpr double kLowTolerance = 0.4;
  ASSERT_THAT(Vector3d(1.0, 2.0, 3.0), Vector3Near({1.0, 2.5, 3.0}, kHighTolerance));
  ASSERT_THAT(Vector3d(1.0, 2.0, 3.0), Not(Vector3Near({1.0, 2.5, 3.0}, kLowTolerance)));
}

TEST(SophusMatchers, SO2Near) {
  constexpr double kTolerance = 0.1;
  ASSERT_THAT(SO2d(1.0, 0.0), SO2Near({1.0, 0.0}, kTolerance));
  ASSERT_THAT(SO2d(1.0, 0.0), Not(SO2Near({0.0, 1.0}, kTolerance)));
}

TEST(SophusMatchers, SE2Near) {
  constexpr double kHighTolerance = 0.6;
  constexpr double kLowTolerance = 0.4;
  ASSERT_THAT(SE2d(SO2d(1.0, 0.0), Vector2d(1.0, 2.0)), SE2Near({1.0, 0.0}, {1.5, 2.5}, kHighTolerance));
  ASSERT_THAT(SE2d(SO2d(1.0, 0.0), Vector2d(1.0, 2.0)), Not(SE2Near({1.0, 0.0}, {1.5, 2.5}, kLowTolerance)));
  ASSERT_THAT(SE2d(SO2d(1.0, 0.0), Vector2d(1.0, 2.0)), Not(SE2Near({0.0, 1.0}, {1.5, 2.5}, kHighTolerance)));
}

}  // namespace

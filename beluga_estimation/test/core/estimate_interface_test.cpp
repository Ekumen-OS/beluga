// Copyright 2026 Ekumen, Inc.
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

/// @file
/// @brief Compilation and contract tests for estimate/store interfaces.
///
/// These tests verify that the headers compile, that the mock types are
/// usable, and that the interfaces accept the documented parameter types.

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/core/estimate.hpp>
#include <beluga_estimation/core/estimate_store.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/mocks/mock_estimate.hpp>

namespace {

// ---------------------------------------------------------------------------
// MockEstimate — interface testability
// ---------------------------------------------------------------------------

TEST(MockEstimateTest, CanInstantiateMock) {
  beluga_estimation::MockEstimate mock;

  EXPECT_CALL(mock, GetVariables())
      .WillOnce(
          testing::Return(beluga_estimation::view<std::shared_ptr<const beluga_estimation::Variable_<double>>>{}));

  auto vars = mock.GetVariables();
  EXPECT_TRUE(vars.empty());
}

TEST(MockEstimateTest, MarginalizeReturnsNullDefault) {
  beluga_estimation::MockEstimate mock;

  auto tp = beluga_estimation::TimePoint{std::chrono::seconds(10)};
  beluga_estimation::Identifier id{.name = "pose", .timestamp = tp};
  EXPECT_CALL(mock, DoMarginalize(testing::_)).WillOnce(testing::Return(nullptr));

  auto result = mock.Marginalize({&id, 1});
  EXPECT_EQ(result, nullptr);
}

TEST(MockEstimateTest, MarginalizeSingleIdentifier) {
  beluga_estimation::MockEstimate mock;

  beluga_estimation::Identifier id{.name = "pose"};
  auto expected = std::make_shared<beluga_estimation::MockEstimate>();
  EXPECT_CALL(mock, DoMarginalize(testing::_)).WillOnce(testing::Return(expected));

  auto result = mock.Marginalize(id);
  EXPECT_EQ(result, expected);
}

// ---------------------------------------------------------------------------
// MatchScope enum — both values are distinct
// ---------------------------------------------------------------------------

TEST(MatchScopeTest, BothScopesAreDistinct) {
  auto exact = static_cast<int>(beluga_estimation::MatchScope::kExact);
  auto semantic = static_cast<int>(beluga_estimation::MatchScope::kSemantic);
  EXPECT_NE(exact, semantic);
}

}  // namespace

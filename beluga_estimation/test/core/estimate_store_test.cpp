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

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/estimate_store.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/core/on_manifold_estimate.hpp>
#include <beluga_estimation/mocks/mock_estimate.hpp>
#include <beluga_estimation/mocks/mock_variable.hpp>

namespace beluga_estimation {
namespace {

using ::testing::Return;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

/// Convenience: create an Identifier with given name and epoch-offset timestamp.
Identifier Id(const std::string& name, std::int64_t t = 0) {
  return Identifier{name, TimePoint{std::chrono::seconds(t)}};
}

/// Creates a shared_ptr to a mock estimate containing variables with the given IDs.
///
/// Each identifier becomes a MockVariable_ whose @ref Identifiable::Id carries
/// the given identifier. The mock's @ref Estimate_::GetVariables returns a view
/// over these variables, which outlives the returned shared_ptr via capture.
auto MakeEstimate(std::vector<Identifier> ids) {
  std::vector<std::shared_ptr<const Variable_<double>>> variables;
  variables.reserve(ids.size());
  for (auto& id : ids) {
    variables.push_back(std::make_shared<MockVariable_<double>>(std::move(id)));
  }

  auto mock = std::make_shared<testing::NiceMock<MockEstimate_<double>>>();
  ON_CALL(*mock, GetVariables()).WillByDefault(testing::Invoke([variables]() {
    return beluga_estimation::view<std::shared_ptr<const Variable_<double>>>(variables);
  }));

  return mock;
}

// -----------------------------------------------------------------------------
// Construction and Size
// -----------------------------------------------------------------------------

TEST(EstimateStoreImplTest, InitiallyEmpty) {
  EstimateStore_<double> store;
  EXPECT_EQ(store.Size(), 0u);
}

TEST(EstimateStoreImplTest, ClearEmptiesStore) {
  EstimateStore_<double> store;
  store.Add(Id("a"), MakeEstimate({Id("a")}), MatchScope::kExact);
  EXPECT_EQ(store.Size(), 1u);
  store.Clear();
  EXPECT_EQ(store.Size(), 0u);
}

// -----------------------------------------------------------------------------
// Insert by Identifier — kExact
// -----------------------------------------------------------------------------

TEST(EstimateStoreImplTest, InsertByIdNewEntry) {
  EstimateStore_<double> store;
  bool replaced = store.Add(Id("x"), MakeEstimate({Id("x")}), MatchScope::kExact);
  EXPECT_FALSE(replaced);
  EXPECT_EQ(store.Size(), 1u);
}

TEST(EstimateStoreImplTest, InsertByIdReplacesExisting) {
  EstimateStore_<double> store;
  auto first = MakeEstimate({Id("x")});
  auto second = MakeEstimate({Id("x")});
  store.Add(Id("x"), first, MatchScope::kExact);
  bool replaced = store.Add(Id("x"), second, MatchScope::kExact);
  EXPECT_TRUE(replaced);
  EXPECT_EQ(store.Size(), 1u);

  auto result = store.Get(Id("x"));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, second);
}

TEST(EstimateStoreImplTest, InsertByIdNullptrThrows) {
  EstimateStore_<double> store;
  EXPECT_THROW(store.Add(Id("x"), nullptr, MatchScope::kExact), std::runtime_error);
}

// -----------------------------------------------------------------------------
// Insert by Identifier — kSemantic
// -----------------------------------------------------------------------------

TEST(EstimateStoreImplTest, InsertByIdSemanticReplacesByName) {
  EstimateStore_<double> store;
  auto first = MakeEstimate({Id("x")});
  auto second = MakeEstimate({Id("x")});
  store.Add(Id("x", 1), first, MatchScope::kExact);
  bool replaced = store.Add(Id("x", 99), second, MatchScope::kSemantic);
  EXPECT_TRUE(replaced);
  EXPECT_EQ(store.Size(), 1u);

  // The old entry with name "x" timestamp 1 should have been removed,
  // and the new entry with timestamp 99 should be present.
  auto result = store.Get(Id("x", 99));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, second);

  EXPECT_EQ(store.Get(Id("x", 1)), nullptr);
}

TEST(EstimateStoreImplTest, InsertByIdSemanticNullptrThrows) {
  EstimateStore_<double> store;
  EXPECT_THROW(store.Add(Id("x"), nullptr, MatchScope::kSemantic), std::runtime_error);
}

TEST(EstimateStoreImplTest, InsertByIdSemanticDoesNotAffectDifferentName) {
  EstimateStore_<double> store;
  store.Add(Id("a", 1), MakeEstimate({Id("a")}), MatchScope::kExact);
  store.Add(Id("b", 1), MakeEstimate({Id("b")}), MatchScope::kExact);
  EXPECT_EQ(store.Size(), 2u);

  // Semantic insert of "a" should not affect "b".
  bool replaced = store.Add(Id("a", 99), MakeEstimate({Id("a")}), MatchScope::kSemantic);
  EXPECT_TRUE(replaced);
  EXPECT_EQ(store.Size(), 2u);

  EXPECT_NE(store.Get(Id("a", 99)), nullptr);
  EXPECT_NE(store.Get(Id("b", 1)), nullptr);
  EXPECT_EQ(store.Get(Id("a", 1)), nullptr);
}

// -----------------------------------------------------------------------------
// Get
// -----------------------------------------------------------------------------

TEST(EstimateStoreImplTest, GetByIdFound) {
  EstimateStore_<double> store;
  auto estimate = MakeEstimate({Id("pose")});
  store.Add(Id("pose"), estimate, MatchScope::kExact);

  auto result = store.Get(Id("pose"));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, estimate);
}

TEST(EstimateStoreImplTest, GetByIdNotFound) {
  EstimateStore_<double> store;
  auto estimate = MakeEstimate({Id("pose")});
  store.Add(Id("pose"), estimate, MatchScope::kExact);

  auto result = store.Get(Id("velocity"));
  EXPECT_EQ(result, nullptr);
}

TEST(EstimateStoreImplTest, GetByNameFound) {
  EstimateStore_<double> store;
  auto estimate = MakeEstimate({Id("pose")});
  store.Add(Id("pose", 10), estimate, MatchScope::kExact);

  auto result = store.GetLatest("pose");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, estimate);
}

TEST(EstimateStoreImplTest, GetByNameNotFound) {
  EstimateStore_<double> store;
  store.Add(Id("pose"), MakeEstimate({Id("pose")}), MatchScope::kExact);

  auto result = store.GetLatest("velocity");
  EXPECT_EQ(result, nullptr);
}

TEST(EstimateStoreImplTest, GetByNameReturnsLatest) {
  EstimateStore_<double> store;
  auto old_est = MakeEstimate({Id("pose")});
  auto new_est = MakeEstimate({Id("pose")});
  store.Add(Id("pose", 10), old_est, MatchScope::kExact);
  store.Add(Id("pose", 99), new_est, MatchScope::kExact);

  auto result = store.GetLatest("pose");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, new_est);
}

// -----------------------------------------------------------------------------
// Typed Get
// -----------------------------------------------------------------------------

TEST(EstimateStoreImplTest, TypedGetByIdReturnsEstimate) {
  EstimateStore_<double> store;
  auto estimate = MakeEstimate({Id("pose")});
  store.Add(Id("pose"), estimate, MatchScope::kExact);

  auto result = store.Get<MockEstimate_<double>>(Id("pose"));
  ASSERT_TRUE(result);
  EXPECT_EQ(Unwrap(result), estimate);
}

TEST(EstimateStoreImplTest, TypedGetByIdNotFoundReturnsNullptr) {
  EstimateStore_<double> store;
  auto result = store.Get<MockEstimate_<double>>(Id("pose"));
  ASSERT_TRUE(result);
  EXPECT_EQ(Unwrap(result), nullptr);
}

TEST(EstimateStoreImplTest, TypedGetByIdTypeMismatchReturnsError) {
  EstimateStore_<double> store;
  store.Add(Id("pose"), MakeEstimate({Id("pose")}), MatchScope::kExact);

  auto result = store.Get<OnManifoldEstimate_<double>>(Id("pose"));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), Error::kFailure);
}

TEST(EstimateStoreImplTest, TypedGetLatestReturnsEstimate) {
  EstimateStore_<double> store;
  auto estimate = MakeEstimate({Id("pose")});
  store.Add(Id("pose", 10), estimate, MatchScope::kExact);

  auto result = store.GetLatest<MockEstimate_<double>>("pose");
  ASSERT_TRUE(result);
  EXPECT_EQ(Unwrap(result), estimate);
}

TEST(EstimateStoreImplTest, TypedGetLatestTypeMismatchReturnsError) {
  EstimateStore_<double> store;
  store.Add(Id("pose"), MakeEstimate({Id("pose")}), MatchScope::kExact);

  auto result = store.GetLatest<OnManifoldEstimate_<double>>("pose");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), Error::kFailure);
}

TEST(EstimateStoreImplTest, TypedGetLatestForVariableReturnsEstimate) {
  EstimateStore_<double> store;
  auto estimate = MakeEstimate({Id("pose")});
  store.Add(Id("pose"), estimate, MatchScope::kExact);

  auto result = store.GetLatestForVariable<MockEstimate_<double>>(Id("pose"));
  ASSERT_TRUE(result);
  EXPECT_EQ(Unwrap(result), estimate);
}

TEST(EstimateStoreImplTest, TypedGetLatestForVariableTypeMismatchReturnsError) {
  EstimateStore_<double> store;
  store.Add(Id("pose"), MakeEstimate({Id("pose")}), MatchScope::kExact);

  auto result = store.GetLatestForVariable<OnManifoldEstimate_<double>>(Id("pose"));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), Error::kFailure);
}

// -----------------------------------------------------------------------------
// Remove by Identifier
// -----------------------------------------------------------------------------

TEST(EstimateStoreImplTest, InterleavedSemanticRemoveAcrossNames) {
  // Regression: semantic remove must find all entries with the same name
  // even when timestamps are interleaved with other names, requiring
  // name-first clustering in the underlying comparator.
  EstimateStore_<double> store;
  store.Add(Id("a", 1), MakeEstimate({Id("a", 1)}), MatchScope::kExact);
  store.Add(Id("b", 50), MakeEstimate({Id("b", 50)}), MatchScope::kExact);
  store.Add(Id("a", 100), MakeEstimate({Id("a", 100)}), MatchScope::kExact);
  EXPECT_EQ(store.Size(), 3u);

  // Semantic remove of "a" must erase BOTH a@1 and a@100, not just the first.
  std::size_t count = store.Remove(Id("a", 1), MatchScope::kSemantic);
  EXPECT_EQ(count, 2u);
  EXPECT_EQ(store.Size(), 1u);
  EXPECT_EQ(store.Get(Id("a", 1)), nullptr);
  EXPECT_EQ(store.Get(Id("a", 100)), nullptr);
  EXPECT_NE(store.Get(Id("b", 50)), nullptr);
}

TEST(EstimateStoreImplTest, GetLatestAfterInterleavedInserts) {
  // After name-clustering inserts, GetLatest("a") must return the newest
  // timestamp for "a", not the first encountered.
  EstimateStore_<double> store;
  auto a1 = MakeEstimate({Id("a", 1)});
  auto b50 = MakeEstimate({Id("b", 50)});
  auto a100 = MakeEstimate({Id("a", 100)});
  store.Add(Id("a", 1), a1, MatchScope::kExact);
  store.Add(Id("b", 50), b50, MatchScope::kExact);
  store.Add(Id("a", 100), a100, MatchScope::kExact);
  EXPECT_EQ(store.Size(), 3u);

  auto result = store.GetLatest("a");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, a100);
}

TEST(EstimateStoreImplTest, RemoveByIdExact) {
  EstimateStore_<double> store;
  store.Add(Id("x"), MakeEstimate({Id("x")}), MatchScope::kExact);
  EXPECT_EQ(store.Size(), 1u);

  std::size_t count = store.Remove(Id("x"), MatchScope::kExact);
  EXPECT_EQ(count, 1u);
  EXPECT_EQ(store.Size(), 0u);

  EXPECT_EQ(store.Get(Id("x")), nullptr);
}

TEST(EstimateStoreImplTest, RemoveByIdNonExisting) {
  EstimateStore_<double> store;
  store.Add(Id("a"), MakeEstimate({Id("a")}), MatchScope::kExact);
  EXPECT_EQ(store.Size(), 1u);

  std::size_t count = store.Remove(Id("b"), MatchScope::kExact);
  EXPECT_EQ(count, 0u);
  EXPECT_EQ(store.Size(), 1u);
}

TEST(EstimateStoreImplTest, RemoveByIdSemantic) {
  EstimateStore_<double> store;
  store.Add(Id("x", 1), MakeEstimate({Id("x")}), MatchScope::kExact);
  store.Add(Id("x", 99), MakeEstimate({Id("x")}), MatchScope::kExact);
  EXPECT_EQ(store.Size(), 2u);

  // Remove with kSemantic — should remove all entries with name "x".
  std::size_t count = store.Remove(Id("x", 1), MatchScope::kSemantic);
  EXPECT_EQ(count, 2u);
  EXPECT_EQ(store.Size(), 0u);
}

TEST(EstimateStoreImplTest, RemoveByIdSemanticDoesNotAffectDifferentName) {
  EstimateStore_<double> store;
  store.Add(Id("a"), MakeEstimate({Id("a")}), MatchScope::kExact);
  store.Add(Id("b"), MakeEstimate({Id("b")}), MatchScope::kExact);
  EXPECT_EQ(store.Size(), 2u);

  // Remove "a" with semantic should not affect "b".
  std::size_t count = store.Remove(Id("a"), MatchScope::kSemantic);
  EXPECT_EQ(count, 1u);
  EXPECT_EQ(store.Size(), 1u);
  EXPECT_NE(store.Get(Id("b")), nullptr);
}

// -----------------------------------------------------------------------------
// Integration
// -----------------------------------------------------------------------------

TEST(EstimateStoreImplTest, InsertGetRemoveSequence) {
  EstimateStore_<double> store;

  // Insert {a}
  auto a_est = MakeEstimate({Id("a")});
  store.Add(Id("a"), a_est, MatchScope::kExact);
  EXPECT_EQ(store.Size(), 1u);

  // Get should find it
  EXPECT_EQ(store.Get(Id("a")), a_est);

  // Remove it
  EXPECT_EQ(store.Remove(Id("a"), MatchScope::kExact), 1u);
  EXPECT_EQ(store.Size(), 0u);
  EXPECT_EQ(store.Get(Id("a")), nullptr);
}

TEST(EstimateStoreImplTest, SemanticInsertThenGetByName) {
  EstimateStore_<double> store;
  auto estimate = MakeEstimate({Id("x")});
  store.Add(Id("x", 10), estimate, MatchScope::kSemantic);

  auto result = store.GetLatest("x");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, estimate);
}

// -----------------------------------------------------------------------------
// GetLatestForVariable
// -----------------------------------------------------------------------------

TEST(EstimateStoreTest, GetLatestForVariableFindsContainingEstimate) {
  EstimateStore_<double> store;
  auto estimate = MakeEstimate({Id("robot_pose")});
  store.Add(Id("pose_est"), estimate);

  auto result = store.GetLatestForVariable(Id("robot_pose"));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, estimate);
}

TEST(EstimateStoreTest, GetLatestForVariableReturnsLatestOfMultiple) {
  EstimateStore_<double> store;
  auto early = MakeEstimate({Id("robot_pose")});
  auto latest = MakeEstimate({Id("robot_pose")});
  store.Add(Id("pose_est", 1), early);
  store.Add(Id("pose_est", 2), latest);

  auto result = store.GetLatestForVariable(Id("robot_pose"));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, latest);
}

TEST(EstimateStoreTest, GetLatestForVariableNoMatchReturnsNull) {
  EstimateStore_<double> store;
  auto estimate = MakeEstimate({Id("something_else")});
  store.Add(Id("other_est"), estimate);

  auto result = store.GetLatestForVariable(Id("robot_pose"));
  EXPECT_EQ(result, nullptr);
}

TEST(EstimateStoreTest, GetLatestForVariableEmptyStoreReturnsNull) {
  EstimateStore_<double> store;
  auto result = store.GetLatestForVariable(Id("robot_pose"));
  EXPECT_EQ(result, nullptr);
}

TEST(EstimateStoreTest, GetLatestForVariableMultiVariableEstimate) {
  EstimateStore_<double> store;
  // Estimate contains variables "x" and "y"; query by "y" should find it.
  auto estimate = MakeEstimate({Id("x"), Id("y")});
  store.Add(Id("multi_est"), estimate);

  auto result = store.GetLatestForVariable(Id("y"));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, estimate);
}

TEST(EstimateStoreTest, GetLatestForVariableEmptyVariablesReturnsNull) {
  EstimateStore_<double> store;
  // Estimate exists but contains no variables — should not match any query.
  auto estimate = MakeEstimate({});
  store.Add(Id("empty_est"), estimate);

  auto result = store.GetLatestForVariable(Id("anything"));
  EXPECT_EQ(result, nullptr);
}

TEST(EstimateStoreTest, GetLatestForVariableRequiresFullIdentifierMatch) {
  EstimateStore_<double> store;
  auto estimate = MakeEstimate({Id("robot_pose", 999)});
  store.Add(Id("pose_est", 1), estimate);

  // Same name but different timestamp → no match (full identifier comparison).
  auto result = store.GetLatestForVariable(Id("robot_pose", 100));
  ASSERT_EQ(result, nullptr);

  // Exact identifier match (name + timestamp) → finds the estimate.
  result = store.GetLatestForVariable(Id("robot_pose", 999));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, estimate);
}

// -----------------------------------------------------------------------------
// GetLatestForVariable — ambiguity (multiple names)
// -----------------------------------------------------------------------------

TEST(EstimateStoreTest, GetLatestForVariableWithMultipleNamesReturnsNull) {
  EstimateStore_<double> store;
  // Two estimates with different names, both contain the same variable.
  auto est1 = MakeEstimate({Id("robot_pose")});
  auto est2 = MakeEstimate({Id("robot_pose")});
  store.Add(Id("kalman", 1), est1);
  store.Add(Id("particle", 1), est2);

  // Multiple names → always ambiguous → nullptr.
  auto result = store.GetLatestForVariable(Id("robot_pose"));
  EXPECT_EQ(result, nullptr);
}

TEST(EstimateStoreTest, GetLatestForVariableWithMultipleNamesDifferentVars) {
  EstimateStore_<double> store;
  // Two estimates with different names and different variables.
  auto est1 = MakeEstimate({Id("pose")});
  auto est2 = MakeEstimate({Id("velocity")});
  store.Add(Id("kalman", 1), est1);
  store.Add(Id("particle", 1), est2);

  // Multiple names → always ambiguous → nullptr, even though
  // the variable only exists in one estimate.
  auto result = store.GetLatestForVariable(Id("pose"));
  EXPECT_EQ(result, nullptr);
}

// -----------------------------------------------------------------------------
// GetNumEstimateNames
// -----------------------------------------------------------------------------

TEST(EstimateStoreTest, GetNumEstimateNamesEmptyStore) {
  EstimateStore_<double> store;
  EXPECT_EQ(store.GetNumEstimateNames(), 0u);
}

TEST(EstimateStoreTest, GetNumEstimateNamesSingleName) {
  EstimateStore_<double> store;
  store.Add(Id("pose_est", 1), MakeEstimate({Id("a")}));
  EXPECT_EQ(store.GetNumEstimateNames(), 1u);

  // Second estimate with the same name should not increase the count.
  store.Add(Id("pose_est", 2), MakeEstimate({Id("b")}));
  EXPECT_EQ(store.GetNumEstimateNames(), 1u);
}

TEST(EstimateStoreTest, GetNumEstimateNamesMultipleNames) {
  EstimateStore_<double> store;
  store.Add(Id("kalman", 1), MakeEstimate({Id("a")}));
  store.Add(Id("particle", 1), MakeEstimate({Id("b")}));
  EXPECT_EQ(store.GetNumEstimateNames(), 2u);

  store.Add(Id("particle", 2), MakeEstimate({Id("c")}));
  EXPECT_EQ(store.GetNumEstimateNames(), 2u);
}

TEST(EstimateStoreTest, GetNumEstimateNamesAfterRemove) {
  EstimateStore_<double> store;
  store.Add(Id("kalman", 1), MakeEstimate({Id("a")}));
  store.Add(Id("particle", 1), MakeEstimate({Id("b")}));

  // Remove the only "particle" entry → count drops to 1.
  store.Remove(Id("particle", 1), MatchScope::kExact);
  EXPECT_EQ(store.GetNumEstimateNames(), 1u);

  // Remove the only "kalman" entry → count drops to 0.
  store.Remove(Id("kalman", 1), MatchScope::kExact);
  EXPECT_EQ(store.GetNumEstimateNames(), 0u);
}

TEST(EstimateStoreTest, GetNumEstimateNamesAfterSemanticRemove) {
  EstimateStore_<double> store;
  store.Add(Id("kalman", 1), MakeEstimate({Id("a")}));
  store.Add(Id("kalman", 2), MakeEstimate({Id("b")}));
  store.Add(Id("particle", 1), MakeEstimate({Id("c")}));

  // Semantic remove of "kalman" removes both entries → count drops to 1.
  store.Remove(Id("kalman", 1), MatchScope::kSemantic);
  EXPECT_EQ(store.GetNumEstimateNames(), 1u);
}

TEST(EstimateStoreTest, GetNumEstimateNamesAfterClear) {
  EstimateStore_<double> store;
  store.Add(Id("kalman", 1), MakeEstimate({Id("a")}));
  store.Add(Id("particle", 1), MakeEstimate({Id("b")}));
  EXPECT_EQ(store.GetNumEstimateNames(), 2u);

  store.Clear();
  EXPECT_EQ(store.GetNumEstimateNames(), 0u);
}

// -----------------------------------------------------------------------------
// GetEstimateNames
// -----------------------------------------------------------------------------

TEST(EstimateStoreTest, GetEstimateNames) {
  EstimateStore_<double> store;
  store.Add(Id("kalman", 1), MakeEstimate({Id("a")}));
  store.Add(Id("particle", 1), MakeEstimate({Id("b")}));

  bool found_kalman = false;
  bool found_particle = false;
  for (const auto& name : store.GetEstimateNames()) {
    if (name == "kalman") {
      found_kalman = true;
    }
    if (name == "particle") {
      found_particle = true;
    }
  }
  EXPECT_TRUE(found_kalman);
  EXPECT_TRUE(found_particle);
}

TEST(EstimateStoreTest, GetEstimateNamesEmpty) {
  EstimateStore_<double> store;
  bool any_iterated = false;
  for ([[maybe_unused]] const auto& name : store.GetEstimateNames()) {
    any_iterated = true;
  }
  EXPECT_FALSE(any_iterated);
}

TEST(EstimateStoreTest, GetEstimateNamesSingleEntry) {
  EstimateStore_<double> store;
  store.Add(Id("pose_est", 1), MakeEstimate({Id("a")}));

  std::size_t count = 0;
  for (const auto& name : store.GetEstimateNames()) {
    EXPECT_EQ(name, "pose_est");
    ++count;
  }
  EXPECT_EQ(count, 1u);
}

}  // namespace
}  // namespace beluga_estimation

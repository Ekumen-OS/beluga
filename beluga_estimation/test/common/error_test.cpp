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
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <beluga_estimation/common/error.hpp>
#include <beluga_estimation/common/result.hpp>

namespace {

// --- Error construction and accessors -------------------------------------

TEST(ErrorTest, DefaultConstructsToFailure) {
  const beluga_estimation::Error error;
  EXPECT_EQ(error.GetCode(), beluga_estimation::Error::kFailure);
  EXPECT_TRUE(error.GetDescription().empty());
  EXPECT_EQ(error.GetCause(), nullptr);
  EXPECT_TRUE(error.GetSideEffects().empty());
}

TEST(ErrorTest, CodeOnlyHasEmptyDescriptionAndNoCause) {
  const beluga_estimation::Error error{beluga_estimation::Error::kTimeout};
  EXPECT_EQ(error.GetCode(), beluga_estimation::Error::kTimeout);
  EXPECT_TRUE(error.GetDescription().empty());
  EXPECT_EQ(error.GetCause(), nullptr);
}

TEST(ErrorTest, DescriptionOnlyDefaultsToFailure) {
  const beluga_estimation::Error error{"something broke"};
  EXPECT_EQ(error.GetCode(), beluga_estimation::Error::kFailure);
  EXPECT_EQ(error.GetDescription(), "something broke");
}

TEST(ErrorTest, CodeAndDescriptionConstructsCorrectly) {
  const beluga_estimation::Error error{beluga_estimation::Error::kFailure, "type 'foo' is not registered"};
  EXPECT_EQ(error.GetCode(), beluga_estimation::Error::kFailure);
  EXPECT_EQ(error.GetDescription(), "type 'foo' is not registered");
}

TEST(ErrorTest, IsMoveOnly) {
  EXPECT_FALSE(std::is_copy_constructible_v<beluga_estimation::Error>);
  EXPECT_FALSE(std::is_copy_assignable_v<beluga_estimation::Error>);
  EXPECT_TRUE(std::is_move_constructible_v<beluga_estimation::Error>);
  EXPECT_TRUE(std::is_move_assignable_v<beluga_estimation::Error>);
}

// --- Cause chaining -------------------------------------------------------

TEST(ErrorTest, ConstructsWithCause) {
  const beluga_estimation::Error error{
      beluga_estimation::Error::kFailure, "finalize failed",
      beluga_estimation::Error(beluga_estimation::Error::kTimeout, "late")};
  EXPECT_EQ(error.GetCode(), beluga_estimation::Error::kFailure);
  ASSERT_NE(error.GetCause(), nullptr);
  EXPECT_EQ(error.GetCause()->GetCode(), beluga_estimation::Error::kTimeout);
  EXPECT_EQ(error.GetCause()->GetDescription(), "late");
}

TEST(ErrorTest, DescriptionAndCauseDefaultsToFailure) {
  const beluga_estimation::Error error{"finalize failed", beluga_estimation::Error(beluga_estimation::Error::kTimeout)};
  EXPECT_EQ(error.GetCode(), beluga_estimation::Error::kFailure);
  ASSERT_NE(error.GetCause(), nullptr);
  EXPECT_EQ(error.GetCause()->GetCode(), beluga_estimation::Error::kTimeout);
}

TEST(ErrorTest, SupportsMultiLevelCauseChains) {
  const beluga_estimation::Error head{
      beluga_estimation::Error::kFailure, "head",
      beluga_estimation::Error(
          beluga_estimation::Error::kFailure, "middle",
          beluga_estimation::Error(beluga_estimation::Error::kTimeout, "root"))};

  const beluga_estimation::Error* cause = head.GetCause();
  ASSERT_NE(cause, nullptr);
  EXPECT_EQ(cause->GetDescription(), "middle");
  ASSERT_NE(cause->GetCause(), nullptr);
  EXPECT_EQ(cause->GetCause()->GetDescription(), "root");
  EXPECT_EQ(cause->GetCause()->GetCause(), nullptr);
}

// --- Side effects ---------------------------------------------------------

TEST(ErrorTest, ConstructorRecordsSideEffects) {
  std::vector<beluga_estimation::Error> effects;
  effects.push_back(beluga_estimation::Error{beluga_estimation::Error::kFailure, "finalize failed"});
  const beluga_estimation::Error error{
      beluga_estimation::Error{beluga_estimation::Error::kTimeout, "run timed out"}, std::move(effects)};

  ASSERT_EQ(error.GetSideEffects().size(), 1U);
  EXPECT_EQ(error.GetSideEffects()[0].GetCode(), beluga_estimation::Error::kFailure);
  EXPECT_EQ(error.GetSideEffects()[0].GetDescription(), "finalize failed");
}

TEST(ErrorTest, ConstructorAppendsAllSideEffects) {
  std::vector<beluga_estimation::Error> effects;
  effects.push_back(beluga_estimation::Error{beluga_estimation::Error::kFailure, "a"});
  effects.push_back(beluga_estimation::Error{beluga_estimation::Error::kFailure, "b"});
  const beluga_estimation::Error error{
      beluga_estimation::Error{beluga_estimation::Error::kTimeout}, std::move(effects)};

  ASSERT_EQ(error.GetSideEffects().size(), 2U);
  EXPECT_EQ(error.GetSideEffects()[0].GetDescription(), "a");
  EXPECT_EQ(error.GetSideEffects()[1].GetDescription(), "b");
}

TEST(ErrorTest, ConstructorAppendsToExistingSideEffects) {
  std::vector<beluga_estimation::Error> first;
  first.push_back(beluga_estimation::Error{beluga_estimation::Error::kFailure, "a"});
  std::vector<beluga_estimation::Error> more;
  more.push_back(beluga_estimation::Error{beluga_estimation::Error::kFailure, "b"});
  beluga_estimation::Error error{beluga_estimation::Error{beluga_estimation::Error::kTimeout}, std::move(first)};
  const beluga_estimation::Error combined{std::move(error), std::move(more)};

  ASSERT_EQ(combined.GetSideEffects().size(), 2U);
  EXPECT_EQ(combined.GetSideEffects()[0].GetDescription(), "a");
  EXPECT_EQ(combined.GetSideEffects()[1].GetDescription(), "b");
}

TEST(ErrorTest, ConstructorReplacesCause) {
  beluga_estimation::Error head{
      beluga_estimation::Error::kFailure, "head", beluga_estimation::Error(beluga_estimation::Error::kBusy)};
  const beluga_estimation::Error chained{
      std::move(head), beluga_estimation::Error(beluga_estimation::Error::kTimeout, "prior")};

  ASSERT_NE(chained.GetCause(), nullptr);
  EXPECT_EQ(chained.GetCause()->GetCode(), beluga_estimation::Error::kTimeout);
  EXPECT_EQ(chained.GetCause()->GetDescription(), "prior");
}

TEST(ErrorTest, SideEffectsCanThemselvesCarryChainsAndEffects) {
  std::vector<beluga_estimation::Error> nested;
  nested.push_back(beluga_estimation::Error{beluga_estimation::Error::kFailure, "nested"});
  beluga_estimation::Error effect{
      beluga_estimation::Error{
          beluga_estimation::Error::kFailure, "cleanup",
          beluga_estimation::Error(beluga_estimation::Error::kTimeout, "root")},
      std::move(nested)};
  std::vector<beluga_estimation::Error> effects;
  effects.push_back(std::move(effect));
  const beluga_estimation::Error error{
      beluga_estimation::Error{beluga_estimation::Error::kTimeout}, std::move(effects)};

  ASSERT_EQ(error.GetSideEffects().size(), 1U);
  const auto& recorded = error.GetSideEffects()[0];
  ASSERT_NE(recorded.GetCause(), nullptr);
  EXPECT_EQ(recorded.GetCause()->GetDescription(), "root");
  ASSERT_EQ(recorded.GetSideEffects().size(), 1U);
  EXPECT_EQ(recorded.GetSideEffects()[0].GetDescription(), "nested");
}

// --- Equality -------------------------------------------------------------

TEST(ErrorTest, EqualityComparesCodeAndDescription) {
  EXPECT_EQ(
      (beluga_estimation::Error{beluga_estimation::Error::kTimeout}),
      (beluga_estimation::Error{beluga_estimation::Error::kTimeout}));
  EXPECT_NE(
      (beluga_estimation::Error{beluga_estimation::Error::kTimeout}),
      (beluga_estimation::Error{beluga_estimation::Error::kInterrupted}));
  EXPECT_NE(
      (beluga_estimation::Error{beluga_estimation::Error::kTimeout, "x"}),
      (beluga_estimation::Error{beluga_estimation::Error::kTimeout, "y"}));
}

TEST(ErrorTest, EqualityIgnoresCauseAndSideEffects) {
  std::vector<beluga_estimation::Error> effects;
  effects.push_back(beluga_estimation::Error{beluga_estimation::Error::kBusy});
  const beluga_estimation::Error a{
      beluga_estimation::Error{
          beluga_estimation::Error::kFailure, "x", beluga_estimation::Error(beluga_estimation::Error::kTimeout)},
      std::move(effects)};
  const beluga_estimation::Error b{beluga_estimation::Error::kFailure, "x"};
  EXPECT_EQ(a, b);
}

// --- Streaming and formatting ---------------------------------------------

TEST(ErrorTest, StreamingWritesCodeName) {
  std::ostringstream stream;
  stream << beluga_estimation::Error{beluga_estimation::Error::kBusy};
  EXPECT_EQ(stream.str(), "BUSY");
}

TEST(ErrorTest, StreamingIncludesDescription) {
  std::ostringstream stream;
  stream << beluga_estimation::Error{beluga_estimation::Error::kTimeout, "took too long"};
  EXPECT_EQ(stream.str(), "TIMEOUT: took too long");
}

TEST(ErrorTest, StreamingWalksCauseChain) {
  std::ostringstream stream;
  stream << beluga_estimation::Error{
      beluga_estimation::Error::kFailure, "finalize failed",
      beluga_estimation::Error(beluga_estimation::Error::kTimeout, "late")};
  EXPECT_EQ(stream.str(), "FAILURE: finalize failed <- TIMEOUT: late");
}

TEST(ErrorTest, StreamingIncludesSideEffects) {
  std::vector<beluga_estimation::Error> effects;
  effects.push_back(beluga_estimation::Error{beluga_estimation::Error::kFailure, "finalize failed"});
  const beluga_estimation::Error error{
      beluga_estimation::Error{beluga_estimation::Error::kTimeout, "run timed out"}, std::move(effects)};
  std::ostringstream stream;
  stream << error;
  EXPECT_EQ(stream.str(), "TIMEOUT: run timed out [also: FAILURE: finalize failed]");
}

TEST(ErrorTest, FmtFormatterMatchesStreaming) {
  std::vector<beluga_estimation::Error> effects;
  effects.push_back(beluga_estimation::Error{beluga_estimation::Error::kFailure, "cleanup"});
  const beluga_estimation::Error error{
      beluga_estimation::Error{
          beluga_estimation::Error::kFailure, "finalize failed",
          beluga_estimation::Error(beluga_estimation::Error::kTimeout, "late")},
      std::move(effects)};
  std::ostringstream stream;
  stream << error;
  EXPECT_EQ(fmt::format("{}", error), stream.str());
}

// --- Err overload set -----------------------------------------------------

TEST(ErrTest, DefaultsToFailure) {
  const beluga_estimation::Result result = beluga_estimation::Err();
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kFailure);
}

TEST(ErrTest, AcceptsCode) {
  const beluga_estimation::Result result = beluga_estimation::Err(beluga_estimation::Error::kBusy);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kBusy);
}

TEST(ErrTest, AcceptsMessage) {
  const beluga_estimation::Result result = beluga_estimation::Err("something broke");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kFailure);
  EXPECT_EQ(result.error().GetDescription(), "something broke");
}

TEST(ErrTest, AcceptsCodeAndMessage) {
  const beluga_estimation::Result result = beluga_estimation::Err(beluga_estimation::Error::kTimeout, "late");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kTimeout);
  EXPECT_EQ(result.error().GetDescription(), "late");
}

TEST(ErrTest, AcceptsCodeAndCause) {
  const beluga_estimation::Result result = beluga_estimation::Err(
      beluga_estimation::Error::kFailure, beluga_estimation::Error(beluga_estimation::Error::kTimeout));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kFailure);
  ASSERT_NE(result.error().GetCause(), nullptr);
  EXPECT_EQ(result.error().GetCause()->GetCode(), beluga_estimation::Error::kTimeout);
}

TEST(ErrTest, AcceptsMessageAndCause) {
  const beluga_estimation::Result result =
      beluga_estimation::Err("finalize failed", beluga_estimation::Error(beluga_estimation::Error::kTimeout));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kFailure);
  EXPECT_EQ(result.error().GetDescription(), "finalize failed");
  ASSERT_NE(result.error().GetCause(), nullptr);
  EXPECT_EQ(result.error().GetCause()->GetCode(), beluga_estimation::Error::kTimeout);
}

TEST(ErrTest, AcceptsCodeMessageAndCause) {
  const beluga_estimation::Result result = beluga_estimation::Err(
      beluga_estimation::Error::kFailure, "finalize failed",
      beluga_estimation::Error(beluga_estimation::Error::kTimeout));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kFailure);
  EXPECT_EQ(result.error().GetDescription(), "finalize failed");
  ASSERT_NE(result.error().GetCause(), nullptr);
}

TEST(ErrTest, AcceptsValueResult) {
  const beluga_estimation::Result_<int> result = beluga_estimation::Err(beluga_estimation::Error::kFailure, "nope");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().GetDescription(), "nope");
}

// --- CollectError / CollectErrors -----------------------------------------

TEST(CollectTest, CollectErrorThrowsOnSuccess) {
  beluga_estimation::Result result = beluga_estimation::Ok();
  EXPECT_THROW((void)beluga_estimation::CollectError(result), std::runtime_error);
}

TEST(CollectTest, CollectErrorMovesErrorOutOfLvalue) {
  beluga_estimation::Result result = beluga_estimation::Err(beluga_estimation::Error::kTimeout, "late");
  auto error = beluga_estimation::CollectError(result);
  EXPECT_EQ(error.GetCode(), beluga_estimation::Error::kTimeout);
  EXPECT_EQ(error.GetDescription(), "late");
}

TEST(CollectTest, CollectErrorMovesErrorOutOfRvalue) {
  auto error = beluga_estimation::CollectError(
      beluga_estimation::Result{beluga_estimation::Err(beluga_estimation::Error::kBusy)});
  EXPECT_EQ(error.GetCode(), beluga_estimation::Error::kBusy);
}

TEST(CollectTest, CollectErrorsSkipsSuccesses) {
  beluga_estimation::Result ok = beluga_estimation::Ok();
  beluga_estimation::Result failed = beluga_estimation::Err(beluga_estimation::Error::kTimeout, "late");
  auto errors = beluga_estimation::CollectErrors(ok, failed);
  ASSERT_EQ(errors.size(), 1U);
  EXPECT_EQ(errors[0].GetCode(), beluga_estimation::Error::kTimeout);
}

TEST(CollectTest, CollectErrorsPreservesOrder) {
  beluga_estimation::Result first = beluga_estimation::Err(beluga_estimation::Error::kFailure, "first");
  beluga_estimation::Result second = beluga_estimation::Err(beluga_estimation::Error::kFailure, "second");
  auto errors = beluga_estimation::CollectErrors(first, second);
  ASSERT_EQ(errors.size(), 2U);
  EXPECT_EQ(errors[0].GetDescription(), "first");
  EXPECT_EQ(errors[1].GetDescription(), "second");
}

// --- Unwrap ---------------------------------------------------------------

TEST(UnwrapTest, MovesValueOutOfLvalueResult) {
  beluga_estimation::Result_<std::unique_ptr<int>> result = std::make_unique<int>(7);
  auto value = beluga_estimation::Unwrap(result);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 7);
}

TEST(UnwrapTest, MovesValueOutOfRvalueResult) {
  auto value = beluga_estimation::Unwrap(beluga_estimation::Result_<std::unique_ptr<int>>{std::make_unique<int>(9)});
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 9);
}

TEST(UnwrapTest, ThrowsOnFailedResult) {
  beluga_estimation::Result_<int> result = beluga_estimation::Err(beluga_estimation::Error::kFailure, "broken");
  EXPECT_THROW((void)beluga_estimation::Unwrap(result), tl::bad_expected_access<beluga_estimation::Error>);
}

// --- Relocation -----------------------------------------------------------

TEST(RelocationTest, ErrAdoptsErrorVerbatim) {
  beluga_estimation::Result source = beluga_estimation::Err(
      beluga_estimation::Error::kTimeout, "late", beluga_estimation::Error(beluga_estimation::Error::kBusy));
  const beluga_estimation::Result_<int> relocated = beluga_estimation::Err(beluga_estimation::CollectError(source));
  ASSERT_FALSE(relocated);
  EXPECT_EQ(relocated.error(), beluga_estimation::Error::kTimeout);
  EXPECT_EQ(relocated.error().GetDescription(), "late");
  ASSERT_NE(relocated.error().GetCause(), nullptr);
  EXPECT_EQ(relocated.error().GetCause()->GetCode(), beluga_estimation::Error::kBusy);
}

TEST(RelocationTest, ErrAdoptsErrorAndRecordsSideEffects) {
  beluga_estimation::Result source = beluga_estimation::Err(beluga_estimation::Error::kTimeout, "late");
  beluga_estimation::Result cleanup = beluga_estimation::Err(beluga_estimation::Error::kFailure, "cleanup failed");
  const beluga_estimation::Result_<int> relocated =
      beluga_estimation::Err(beluga_estimation::CollectError(source), beluga_estimation::CollectErrors(cleanup));
  ASSERT_FALSE(relocated);
  EXPECT_EQ(relocated.error(), beluga_estimation::Error::kTimeout);
  ASSERT_EQ(relocated.error().GetSideEffects().size(), 1U);
  EXPECT_EQ(relocated.error().GetSideEffects()[0].GetDescription(), "cleanup failed");
}

TEST(RelocationTest, ErrChainsPriorErrorAsCause) {
  beluga_estimation::Result first = beluga_estimation::Err(beluga_estimation::Error::kTimeout, "first");
  beluga_estimation::Result second = beluga_estimation::Err(beluga_estimation::Error::kFailure, "second");
  const beluga_estimation::Result_<int> chained =
      beluga_estimation::Err(beluga_estimation::CollectError(second), beluga_estimation::CollectError(first));
  ASSERT_FALSE(chained);
  EXPECT_EQ(chained.error(), beluga_estimation::Error::kFailure);
  EXPECT_EQ(chained.error().GetDescription(), "second");
  ASSERT_NE(chained.error().GetCause(), nullptr);
  EXPECT_EQ(chained.error().GetCause()->GetCode(), beluga_estimation::Error::kTimeout);
  EXPECT_EQ(chained.error().GetCause()->GetDescription(), "first");
}

}  // namespace

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

#include <stdexcept>
#include <string>
#include <type_traits>

#include <gtest/gtest.h>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/result.hpp>

namespace {

// --- BELUGA_ESTIMATION_DEMAND --------------------------------------------------------

TEST(BelugaDemandTest, DoesNotThrowWhenConditionHolds) {
  EXPECT_NO_THROW({ BELUGA_ESTIMATION_DEMAND(true); });
  int value = 42;
  EXPECT_NO_THROW({ BELUGA_ESTIMATION_DEMAND(value == 42); });
}

TEST(BelugaDemandTest, ThrowsRuntimeErrorWhenConditionFails) {
  EXPECT_THROW({ BELUGA_ESTIMATION_DEMAND(false); }, std::runtime_error);
  EXPECT_THROW(
      {
        int value = 0;
        BELUGA_ESTIMATION_DEMAND(value != 0);
      },
      std::runtime_error);
}

TEST(BelugaDemandTest, MessageContainsConditionAndSourceLocation) {
  try {
    BELUGA_ESTIMATION_DEMAND(1 == 2);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("1 == 2"), std::string::npos) << message;
    EXPECT_NE(message.find("assert_test.cpp"), std::string::npos) << message;
    EXPECT_NE(message.find("BELUGA_ESTIMATION_DEMAND"), std::string::npos) << message;
  }
}

TEST(BelugaDemandTest, MessageIncludesFormattedArguments) {
  int value = 3;
  try {
    BELUGA_ESTIMATION_DEMAND(value > 10, "got {}, expected > 10", value);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("got 3, expected > 10"), std::string::npos) << message;
  }
}

TEST(BelugaDemandTest, DoesNotEvaluateFormatArgumentsWhenConditionHolds) {
  int calls = 0;
  const auto side_effect = [&calls]() {
    ++calls;
    return 0;
  };
  EXPECT_NO_THROW({ BELUGA_ESTIMATION_DEMAND(true, "side effect {}", side_effect()); });
  EXPECT_EQ(calls, 0);
}

TEST(BelugaDemandTest, EvaluatesConditionExactlyOnceWhenItHolds) {
  int evaluations = 0;
  const auto truthy = [&evaluations]() {
    ++evaluations;
    return true;
  };
  EXPECT_NO_THROW({ BELUGA_ESTIMATION_DEMAND(truthy()); });
  EXPECT_EQ(evaluations, 1);
}

TEST(BelugaDemandTest, DoesNotCaptureVariablesUsedByTheImplementation) {
  const bool condition = true;
  const bool condition_value = true;
  const beluga_estimation::Result result = beluga_estimation::Ok();
  EXPECT_NO_THROW({ BELUGA_ESTIMATION_DEMAND(condition); });
  EXPECT_NO_THROW({ BELUGA_ESTIMATION_DEMAND(condition_value); });
  EXPECT_NO_THROW({ BELUGA_ESTIMATION_DEMAND(result); });
}

TEST(BelugaDemandTest, EvaluatesConditionExactlyOnceWhenItFails) {
  int evaluations = 0;
  const auto falsy = [&evaluations]() {
    ++evaluations;
    return false;
  };
  EXPECT_THROW({ BELUGA_ESTIMATION_DEMAND(falsy()); }, std::runtime_error);
  EXPECT_EQ(evaluations, 1);
}

// --- BELUGA_ESTIMATION_DEMAND on a Result --------------------------------------------

TEST(BelugaDemandResultTest, DoesNotThrowWhenResultHoldsValue) {
  const beluga_estimation::Result result = beluga_estimation::Ok();
  EXPECT_NO_THROW({ BELUGA_ESTIMATION_DEMAND(result); });
}

TEST(BelugaDemandResultTest, MessageIncludesBundledError) {
  const beluga_estimation::Result result = beluga_estimation::Err(beluga_estimation::Error::kTimeout, "late");
  try {
    BELUGA_ESTIMATION_DEMAND(result);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("TIMEOUT: late"), std::string::npos) << message;
    EXPECT_NE(message.find("result"), std::string::npos) << message;
  }
}

TEST(BelugaDemandResultTest, MessageIncludesCauseChainAndSideEffects) {
  std::vector<beluga_estimation::Error> effects;
  effects.push_back(beluga_estimation::Error{beluga_estimation::Error::kFailure, "cleanup"});
  beluga_estimation::Error error{
      beluga_estimation::Error{
          beluga_estimation::Error::kFailure, "finalize failed",
          beluga_estimation::Error(beluga_estimation::Error::kTimeout, "late")},
      std::move(effects)};
  const beluga_estimation::Result result = beluga_estimation::Err(std::move(error));
  try {
    BELUGA_ESTIMATION_DEMAND(result);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("FAILURE: finalize failed <- TIMEOUT: late"), std::string::npos) << message;
    EXPECT_NE(message.find("[also: FAILURE: cleanup]"), std::string::npos) << message;
  }
}

TEST(BelugaDemandResultTest, MessageCombinesBundledErrorAndContext) {
  const beluga_estimation::Result result = beluga_estimation::Err(beluga_estimation::Error::kBusy);
  try {
    BELUGA_ESTIMATION_DEMAND(result, "while loading stage {}", 3);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("BUSY"), std::string::npos) << message;
    EXPECT_NE(message.find("while loading stage 3"), std::string::npos) << message;
  }
}

TEST(BelugaDemandResultTest, EvaluatesConditionExactlyOnceWhenItHolds) {
  int evaluations = 0;
  const auto result = [&evaluations]() -> beluga_estimation::Result {
    ++evaluations;
    return beluga_estimation::Ok();
  };
  EXPECT_NO_THROW({ BELUGA_ESTIMATION_DEMAND(result()); });
  EXPECT_EQ(evaluations, 1);
}

TEST(BelugaDemandResultTest, EvaluatesConditionExactlyOnceWhenItFails) {
  int evaluations = 0;
  const auto result = [&evaluations]() -> beluga_estimation::Result {
    ++evaluations;
    return beluga_estimation::Err(beluga_estimation::Error::kBusy);
  };
  // A failing Result condition is the interesting case: the failure path reads
  // the error back to report it, which must not re-evaluate the expression.
  try {
    BELUGA_ESTIMATION_DEMAND(result());
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("BUSY"), std::string::npos) << message;
  }
  EXPECT_EQ(evaluations, 1);
}

// --- BELUGA_ESTIMATION_ASSERT introspection ------------------------------------------

TEST(BelugaAssertTest, IntrospectionConstantsAreConsistent) {
  EXPECT_TRUE((std::is_same_v<decltype(beluga_estimation::kBelugaEstimationAssertIsArmed), const bool>));
  EXPECT_TRUE((std::is_same_v<decltype(beluga_estimation::kBelugaEstimationAssertIsDisarmed), const bool>));
  EXPECT_NE(beluga_estimation::kBelugaEstimationAssertIsArmed, beluga_estimation::kBelugaEstimationAssertIsDisarmed);
}

}  // namespace

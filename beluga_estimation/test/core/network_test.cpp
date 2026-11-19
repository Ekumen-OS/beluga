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

#include <algorithm>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <range/v3/algorithm/count_if.hpp>
#include <range/v3/algorithm/find.hpp>
#include <range/v3/iterator/operations.hpp>
#include <range/v3/range/conversion.hpp>

#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/core/network_visitor.hpp>
#include <beluga_estimation/mocks/mock_constraint.hpp>
#include <beluga_estimation/mocks/mock_distribution.hpp>
#include <beluga_estimation/mocks/mock_manifold.hpp>
#include <beluga_estimation/mocks/mock_variable.hpp>

namespace {

using namespace beluga_estimation;
using namespace std::chrono_literals;
using ::testing::Return;
using ::testing::ReturnRef;

// Concrete variable classes for testing safe downcasting.
template <class T>
class CustomVariable_ : public MockVariable_<T> {
 public:
  using MockVariable_<T>::MockVariable_;
};

template <class T>
class CustomConstraint_ : public MockConstraint_<T> {
 public:
  using MockConstraint_<T>::MockConstraint_;
};

template <class T>
class MockVisitor_ : public DynamicBayesianNetworkVisitor_<T> {
 public:
  MOCK_METHOD(void, Visit, (const Variable_<T>&), (override));
  MOCK_METHOD(void, Visit, (const Constraint_<T>&), (override));
  MOCK_METHOD(void, Visit, (const AbstractValue&), (override));
};

template <class T>
class DynamicBayesianNetworkTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Add variables
    for (int i = 0; i < 5; ++i) {
      auto variable = std::make_shared<MockVariable_<T>>(Identifier{"var", TimePoint(std::chrono::seconds(i))});
      ASSERT_TRUE(network_.Add(variable));
      variables_.push_back(variable);
    }
    // Add custom variables for downcasting tests
    for (int i = 5; i < 7; ++i) {
      auto variable =
          std::make_shared<CustomVariable_<T>>(Identifier{"custom_var", TimePoint(std::chrono::seconds(i))});
      ASSERT_TRUE(network_.Add(variable));
      variables_.push_back(variable);
    }

    // Add constraints
    for (int i = 0; i < 3; ++i) {
      auto constraint =
          std::make_shared<MockConstraint_<T>>(Identifier{"constraint", TimePoint(std::chrono::seconds(i))});
      const std::vector<Identifier> dependent_variables{variables_[i + 1]->Id()};
      const std::vector<Identifier> conditioning_variables{variables_[i]->Id()};
      ON_CALL(*constraint, HasDependentVariables()).WillByDefault(Return(true));
      ON_CALL(*constraint, HasConditioningVariables()).WillByDefault(Return(true));
      ON_CALL(*constraint, DependentVariables()).WillByDefault(Return(dependent_variables));
      ON_CALL(*constraint, ConditioningVariables()).WillByDefault(Return(conditioning_variables));
      EXPECT_CALL(*constraint, HasDependentVariables()).Times(::testing::AnyNumber());
      EXPECT_CALL(*constraint, HasConditioningVariables()).Times(::testing::AnyNumber());
      EXPECT_CALL(*constraint, DependentVariables()).Times(::testing::AnyNumber());
      EXPECT_CALL(*constraint, ConditioningVariables()).Times(::testing::AnyNumber());
      ASSERT_TRUE(network_.Add(constraint));
      constraints_.push_back(constraint);
    }
    // Add custom constraints for downcasting tests
    for (int i = 3; i < 5; ++i) {
      auto constraint =
          std::make_shared<CustomConstraint_<T>>(Identifier{"custom_constraint", TimePoint(std::chrono::seconds(i))});
      const std::vector<Identifier> dependent_variables{variables_[i + 1]->Id()};
      const std::vector<Identifier> conditioning_variables{variables_[i]->Id()};
      ON_CALL(*constraint, HasDependentVariables()).WillByDefault(Return(true));
      ON_CALL(*constraint, HasConditioningVariables()).WillByDefault(Return(true));
      ON_CALL(*constraint, DependentVariables()).WillByDefault(Return(dependent_variables));
      ON_CALL(*constraint, ConditioningVariables()).WillByDefault(Return(conditioning_variables));
      EXPECT_CALL(*constraint, HasDependentVariables()).Times(::testing::AnyNumber());
      EXPECT_CALL(*constraint, HasConditioningVariables()).Times(::testing::AnyNumber());
      EXPECT_CALL(*constraint, DependentVariables()).Times(::testing::AnyNumber());
      EXPECT_CALL(*constraint, ConditioningVariables()).Times(::testing::AnyNumber());
      ASSERT_TRUE(network_.Add(constraint));
      constraints_.push_back(constraint);
    }

    // Add values
    for (int i = 0; i < 4; ++i) {
      auto value = std::make_shared<Value<int>>(Identifier{"value", TimePoint(std::chrono::seconds(i))}, i);
      ASSERT_TRUE(network_.Add(value));
      values_.push_back(value);
    }
  }

  DynamicBayesianNetwork_<T> network_;
  MockManifold_<T> manifold_;
  MockDistribution_<T> distribution_;
  std::vector<std::shared_ptr<MockVariable_<T>>> variables_;
  std::vector<std::shared_ptr<MockConstraint_<T>>> constraints_;
  std::vector<std::shared_ptr<AbstractValue>> values_;
};

using ValueTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(DynamicBayesianNetworkTest, ValueTypes);

// Variable related tests
TYPED_TEST(DynamicBayesianNetworkTest, GetVariable) {
  auto variable = this->variables_.front();
  ASSERT_EQ(this->network_.GetVariable(variable->Id()), variable);
  ASSERT_EQ(this->network_.GetVariable(Identifier{"not_found"}), nullptr);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetVariableDowncasted) {
  auto custom_variable = this->network_.template GetVariable<CustomVariable_<TypeParam>>(this->variables_.back()->Id());
  ASSERT_TRUE(custom_variable);
  ASSERT_NE(Unwrap(custom_variable), nullptr);

  // A stored variable of a different type is reported as an error.
  auto mismatched = this->network_.template GetVariable<CustomVariable_<TypeParam>>(this->variables_.front()->Id());
  ASSERT_FALSE(mismatched);
  EXPECT_EQ(mismatched.error(), Error::kFailure);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetLatestVariable) {
  auto latest_variable = this->network_.GetLatestVariable("var");
  ASSERT_EQ(latest_variable->Id(), this->variables_[4]->Id());
}

TYPED_TEST(DynamicBayesianNetworkTest, GetLatestVariableAtTime) {
  auto latest_variable = this->network_.GetLatestVariable("var", TimePoint(std::chrono::seconds(2)));
  ASSERT_EQ(latest_variable->Id(), this->variables_[2]->Id());
}

TYPED_TEST(DynamicBayesianNetworkTest, GetAllVariables) {
  ASSERT_EQ(ranges::distance(this->network_.GetVariables()), 7);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetVariablesByName) {
  ASSERT_EQ(ranges::distance(this->network_.GetVariables("var")), 5);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetVariablesByInterval) {
  // [1, 3] — both bounds closed: variables at t=1, 2, 3.
  TimeInterval interval = TimeInterval::Closed(TimePoint(1s), TimePoint(3s));
  ASSERT_EQ(ranges::distance(this->network_.GetVariables(interval)), 3);  // vars at 1, 2, 3
}

TYPED_TEST(DynamicBayesianNetworkTest, GetVariablesByNameAndInterval) {
  // [4, 6] — both bounds closed: custom_vars at t=5, 6.
  TimeInterval interval = TimeInterval::Closed(TimePoint(4s), TimePoint(6s));
  auto variables = this->network_.GetVariables("custom_var", interval);
  ASSERT_EQ(ranges::distance(variables), 2);  // custom_vars at 5, 6
}

TYPED_TEST(DynamicBayesianNetworkTest, GetVariablesByOpenInterval) {
  // (1, 3) — both bounds open: only the variable at t=2.
  TimeInterval interval = TimeInterval::Open(TimePoint(1s), TimePoint(3s));
  ASSERT_EQ(ranges::distance(this->network_.GetVariables(interval)), 1);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetVariablesByHalfOpenInterval) {
  // [1, 3) — lower closed, upper open: variables at t=1, 2.
  TimeInterval interval = TimeInterval::RightOpen(TimePoint(1s), TimePoint(3s));
  ASSERT_EQ(ranges::distance(this->network_.GetVariables(interval)), 2);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetVariablesByHalfOpenIntervalRightClosed) {
  // (1, 3] — lower open, upper closed: variables at t=2, 3.
  TimeInterval interval = TimeInterval::LeftOpen(TimePoint(1s), TimePoint(3s));
  ASSERT_EQ(ranges::distance(this->network_.GetVariables(interval)), 2);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetVariablesUntil) {
  // Until(2) — open upper bound: variables strictly before t=2 (t=0, 1).
  auto variables = this->network_.GetVariables(TimeInterval::Until(TimePoint(std::chrono::seconds(2))));
  ASSERT_EQ(ranges::distance(variables), 2);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetVariablesSince) {
  // Since(2) — open lower bound: variables at t=3, 4, 5, 6.
  auto variables = this->network_.GetVariables(TimeInterval::Since(TimePoint(std::chrono::seconds(2))));
  ASSERT_EQ(ranges::distance(variables), 4);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetVariablesDowncasted) {
  auto variables = this->network_.template GetVariables<CustomVariable_<TypeParam>>("custom_var");
  ASSERT_EQ(ranges::distance(variables), 2);
  for (auto&& var : variables) {
    ASSERT_TRUE(var);
    ASSERT_NE(Unwrap(var), nullptr);
  }
}

// Constraint related tests
TYPED_TEST(DynamicBayesianNetworkTest, GetConstraint) {
  auto constraint = this->constraints_.front();
  ASSERT_EQ(this->network_.GetConstraint(constraint->Id()), constraint);
  ASSERT_EQ(this->network_.GetConstraint(Identifier{"not_found"}), nullptr);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetConstraintDowncasted) {
  auto custom_constraint =
      this->network_.template GetConstraint<CustomConstraint_<TypeParam>>(this->constraints_.back()->Id());
  ASSERT_TRUE(custom_constraint);
  ASSERT_NE(Unwrap(custom_constraint), nullptr);

  // A stored constraint of a different type is reported as an error.
  auto mismatched =
      this->network_.template GetConstraint<CustomConstraint_<TypeParam>>(this->constraints_.front()->Id());
  ASSERT_FALSE(mismatched);
  EXPECT_EQ(mismatched.error(), Error::kFailure);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetLatestConstraint) {
  auto latest_constraint = this->network_.GetLatestConstraint("constraint");
  ASSERT_EQ(latest_constraint->Id(), this->constraints_[2]->Id());
}

TYPED_TEST(DynamicBayesianNetworkTest, GetLatestConstraintAtTime) {
  auto latest_constraint = this->network_.GetLatestConstraint("constraint", TimePoint(std::chrono::seconds(1)));
  ASSERT_EQ(latest_constraint->Id(), this->constraints_[1]->Id());
}

TYPED_TEST(DynamicBayesianNetworkTest, GetAllConstraints) {
  ASSERT_EQ(ranges::distance(this->network_.GetConstraints()), 5);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetConstraintsByName) {
  ASSERT_EQ(ranges::distance(this->network_.GetConstraints("custom_constraint")), 2);
}

// Value related tests
TYPED_TEST(DynamicBayesianNetworkTest, GetValue) {
  auto value = this->values_.front();
  ASSERT_EQ(this->network_.GetValue(value->Id()), value);
  ASSERT_EQ(this->network_.GetValue(Identifier{"not_found"}), nullptr);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetTypedValue) {
  auto value = this->values_.front();
  auto data = this->network_.template GetValue<int>(value->Id());
  ASSERT_TRUE(data);
  auto data_data = Unwrap(data);
  ASSERT_TRUE(data_data.has_value());
  ASSERT_EQ(data_data.value(), 0);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetTypedValueMissingReturnsEmpty) {
  auto data = this->network_.template GetValue<int>(Identifier{"not_found"});
  ASSERT_TRUE(data);
  EXPECT_FALSE(Unwrap(data).has_value());
}

TYPED_TEST(DynamicBayesianNetworkTest, GetTypedValueTypeMismatchReturnsError) {
  auto data = this->network_.template GetValue<std::string>(this->values_.front()->Id());
  ASSERT_FALSE(data);
  EXPECT_EQ(data.error(), Error::kFailure);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetLatestValue) {
  auto latest_value = this->network_.GetLatestValue("value");
  ASSERT_EQ(latest_value->Id(), this->values_[3]->Id());
}

TYPED_TEST(DynamicBayesianNetworkTest, GetTypedLatestValueReturnsData) {
  auto data = this->network_.template GetLatestValue<int>("value");
  ASSERT_TRUE(data);
  auto typed = Unwrap(data);
  ASSERT_TRUE(typed.has_value());
  EXPECT_EQ(typed.value(), 3);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetTypedLatestValueTypeMismatchReturnsError) {
  auto data = this->network_.template GetLatestValue<std::string>("value");
  ASSERT_FALSE(data);
  EXPECT_EQ(data.error(), Error::kFailure);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetTypedLatestValueMissingReturnsEmpty) {
  auto data = this->network_.template GetLatestValue<int>("not_found");
  ASSERT_TRUE(data);
  EXPECT_FALSE(Unwrap(data).has_value());
}

TYPED_TEST(DynamicBayesianNetworkTest, GetTypedValuesReturnsData) {
  std::vector<int> collected;
  for (auto&& result : this->network_.template GetValues<int>("value")) {
    ASSERT_TRUE(result);
    collected.push_back(Unwrap(result));
  }
  EXPECT_EQ(collected, (std::vector<int>{0, 1, 2, 3}));
}

TYPED_TEST(DynamicBayesianNetworkTest, GetTypedValuesEmptyRange) {
  auto data = this->network_.template GetValues<int>("not_found");
  EXPECT_EQ(ranges::distance(data), 0);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetTypedValuesTypeMismatchReturnsErrors) {
  std::size_t count = 0;
  for (auto&& result : this->network_.template GetValues<std::string>("value")) {
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), Error::kFailure);
    ++count;
  }
  EXPECT_EQ(count, 4u);
}

// Modification APIs
TYPED_TEST(DynamicBayesianNetworkTest, UpdateVariable) {
  auto variable = std::make_shared<MockVariable_<TypeParam>>(this->variables_[0]->Id());
  ASSERT_TRUE(this->network_.Update(variable));
  ASSERT_EQ(this->network_.GetVariable(this->variables_[0]->Id())->Id(), variable->Id());
}

TYPED_TEST(DynamicBayesianNetworkTest, UpdateConstraint) {
  auto prev_constraint = this->constraints_[0];
  EXPECT_CALL(*prev_constraint, DependentVariables());
  EXPECT_CALL(*prev_constraint, ConditioningVariables());
  auto constraint = std::make_shared<MockConstraint_<TypeParam>>(prev_constraint->Id());
  const std::vector<Identifier> dependent_variables{this->variables_[1]->Id()};
  const std::vector<Identifier> conditioning_variables{this->variables_[0]->Id()};
  EXPECT_CALL(*constraint, DependentVariables()).WillOnce(Return(dependent_variables));
  EXPECT_CALL(*constraint, ConditioningVariables()).WillOnce(Return(conditioning_variables));
  ASSERT_TRUE(this->network_.Update(constraint));
  ASSERT_EQ(this->network_.GetConstraint(prev_constraint->Id())->Id(), constraint->Id());
}

TYPED_TEST(DynamicBayesianNetworkTest, UpdateValue) {
  auto value = std::make_shared<Value<int>>(this->values_[0]->Id(), 42);
  ASSERT_TRUE(this->network_.Update(value));
  ASSERT_EQ(this->network_.GetValue(this->values_[0]->Id())->Id(), value->Id());
  auto typed_value = this->network_.template GetValue<int>(this->values_[0]->Id());
  ASSERT_TRUE(typed_value);
  auto typed_value_data = Unwrap(typed_value);
  ASSERT_TRUE(typed_value_data.has_value());
  ASSERT_EQ(typed_value_data.value(), 42);
}

TYPED_TEST(DynamicBayesianNetworkTest, AddValue) {
  auto value = std::make_shared<Value<int>>(Identifier{"new_value"}, 100);
  ASSERT_TRUE(this->network_.Add(value));
  ASSERT_EQ(this->network_.GetValue(Identifier{"new_value"}), value);
}

TYPED_TEST(DynamicBayesianNetworkTest, RemoveValue) {
  auto value_id = this->values_[0]->Id();
  ASSERT_TRUE(this->network_.Remove(this->values_[0]));
  ASSERT_EQ(this->network_.GetValue(value_id), nullptr);
}

TYPED_TEST(DynamicBayesianNetworkTest, AddWithConstructor) {
  const auto variable_id = Identifier{"constructor_var"};
  auto variable = this->network_.template Add<MockVariable_<TypeParam>>(variable_id);
  ASSERT_TRUE(variable);
  ASSERT_NE(Unwrap(variable), nullptr);
  ASSERT_NE(this->network_.GetVariable(variable_id), nullptr);

  const auto value_id = Identifier{"constructor_value"};
  auto value = this->network_.template Add<Value<std::string>>(value_id, "hello");
  ASSERT_TRUE(value);
  ASSERT_NE(Unwrap(value), nullptr);
  ASSERT_NE(this->network_.GetValue(value_id), nullptr);
  auto typed_value = this->network_.template GetValue<std::string>(value_id);
  ASSERT_TRUE(typed_value);
  auto typed_value_data = Unwrap(typed_value);
  ASSERT_TRUE(typed_value_data.has_value());
  ASSERT_EQ(typed_value_data.value(), "hello");
}

TYPED_TEST(DynamicBayesianNetworkTest, AddExistingVariable) {
  auto variable = std::make_shared<MockVariable_<TypeParam>>(this->variables_[0]->Id());
  ASSERT_FALSE(this->network_.Add(variable));
}

TYPED_TEST(DynamicBayesianNetworkTest, RemoveVariable) {
  auto variable = std::make_shared<MockVariable_<TypeParam>>(Identifier{"removable_var"});
  this->network_.Add(variable);
  ASSERT_TRUE(this->network_.Remove(variable));
  ASSERT_EQ(this->network_.GetVariable(variable->Id()), nullptr);
}

TYPED_TEST(DynamicBayesianNetworkTest, RemoveNonExistingVariable) {
  auto variable = std::make_shared<MockVariable_<TypeParam>>(Identifier{"non_existing_var"});
  ASSERT_FALSE(this->network_.Remove(variable));
}

TYPED_TEST(DynamicBayesianNetworkTest, AddConstraint) {
  auto variable1 = std::make_shared<MockVariable_<TypeParam>>(Identifier{"new_var_for_constraint_1"});
  auto variable2 = std::make_shared<MockVariable_<TypeParam>>(Identifier{"new_var_for_constraint_2"});
  auto constraint = std::make_shared<MockConstraint_<TypeParam>>(Identifier{"new_constraint"});
  this->network_.Add(variable1);
  this->network_.Add(variable2);

  const std::vector<Identifier> dependent_variables{variable2->Id()};
  const std::vector<Identifier> conditioning_variables{variable1->Id()};
  EXPECT_CALL(*constraint, HasDependentVariables()).WillRepeatedly(Return(true));
  EXPECT_CALL(*constraint, HasConditioningVariables()).WillRepeatedly(Return(true));
  EXPECT_CALL(*constraint, DependentVariables()).WillRepeatedly(Return(dependent_variables));
  EXPECT_CALL(*constraint, ConditioningVariables()).WillRepeatedly(Return(conditioning_variables));
  ASSERT_TRUE(this->network_.Add(constraint));
  ASSERT_EQ(this->network_.GetConstraint(constraint->Id()), constraint);
}

TYPED_TEST(DynamicBayesianNetworkTest, AddExistingConstraint) {
  auto constraint = std::make_shared<MockConstraint_<TypeParam>>(this->constraints_[0]->Id());
  ASSERT_FALSE(this->network_.Add(constraint));
}

TYPED_TEST(DynamicBayesianNetworkTest, AddConstraintWithNoVariable) {
  auto constraint = std::make_shared<MockConstraint_<TypeParam>>(Identifier{"constraint_no_var"});
  const std::vector<Identifier> conditioning_variables{Identifier{"non_existing_var_1"}};
  EXPECT_CALL(*constraint, HasConditioningVariables()).WillOnce(Return(true));
  EXPECT_CALL(*constraint, ConditioningVariables()).WillOnce(Return(conditioning_variables));
  ASSERT_THROW(this->network_.Add(constraint), std::runtime_error);
}

TYPED_TEST(DynamicBayesianNetworkTest, AddConstraintWithPartiallyValidVariablesLeavesNetworkUnchanged) {
  auto variable = std::make_shared<MockVariable_<TypeParam>>(Identifier{"partially_valid_var"});
  ASSERT_TRUE(this->network_.Add(variable));

  // The first conditioning variable exists, the second does not. The failed
  // insertion must not leave a dangling adjacency entry for the first.
  auto constraint = std::make_shared<MockConstraint_<TypeParam>>(Identifier{"partially_valid_constraint"});
  const std::vector<Identifier> conditioning_variables{variable->Id(), Identifier{"missing_var"}};
  ON_CALL(*constraint, HasConditioningVariables()).WillByDefault(Return(true));
  ON_CALL(*constraint, ConditioningVariables()).WillByDefault(Return(conditioning_variables));
  ON_CALL(*constraint, HasDependentVariables()).WillByDefault(Return(false));

  ASSERT_THROW(this->network_.Add(constraint), std::runtime_error);

  EXPECT_EQ(this->network_.GetConstraint(constraint->Id()), nullptr);
  EXPECT_EQ(ranges::distance(this->network_.GetConditionedConstraints(variable->Id())), 0);
}

TYPED_TEST(DynamicBayesianNetworkTest, RemoveConstraint) {
  auto constraint = this->constraints_.front();
  EXPECT_CALL(*constraint, HasDependentVariables()).Times(::testing::AnyNumber());
  EXPECT_CALL(*constraint, HasConditioningVariables()).Times(::testing::AnyNumber());
  EXPECT_CALL(*constraint, DependentVariables()).Times(::testing::AnyNumber());
  EXPECT_CALL(*constraint, ConditioningVariables()).Times(::testing::AnyNumber());
  ASSERT_TRUE(this->network_.Remove(constraint));
  ASSERT_EQ(this->network_.GetConstraint(constraint->Id()), nullptr);
}

TYPED_TEST(DynamicBayesianNetworkTest, RemoveNonExistingConstraint) {
  auto constraint = std::make_shared<MockConstraint_<TypeParam>>(Identifier{"non_existing_constraint"});
  ASSERT_FALSE(this->network_.Remove(constraint));
}

TYPED_TEST(DynamicBayesianNetworkTest, RemoveVariableWithConstraint) {
  ASSERT_THROW(this->network_.Remove(this->variables_.front()), std::runtime_error);
}

TYPED_TEST(DynamicBayesianNetworkTest, Clear) {
  this->network_.Clear();
  ASSERT_EQ(ranges::distance(this->network_.GetVariables()), 0);
  ASSERT_EQ(ranges::distance(this->network_.GetConstraints()), 0);
  ASSERT_EQ(ranges::distance(this->network_.GetValues()), 0);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddNewVariable) {
  auto variable = std::make_shared<MockVariable_<TypeParam>>(Identifier{"new_var"});
  auto result = this->network_.MaybeAdd(variable);
  ASSERT_EQ(result, variable);
  ASSERT_EQ(this->network_.GetVariable(variable->Id()), variable);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddExistingVariable) {
  auto existing = this->variables_[0];
  auto duplicate = std::make_shared<MockVariable_<TypeParam>>(existing->Id());
  auto result = this->network_.MaybeAdd(duplicate);
  ASSERT_EQ(result, existing);
  ASSERT_NE(result, duplicate);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddNewConstraint) {
  auto var1 = std::make_shared<MockVariable_<TypeParam>>(Identifier{"var1_for_maybe"});
  auto var2 = std::make_shared<MockVariable_<TypeParam>>(Identifier{"var2_for_maybe"});
  this->network_.MaybeAdd(var1);
  this->network_.MaybeAdd(var2);

  auto constraint = std::make_shared<MockConstraint_<TypeParam>>(Identifier{"new_constraint_maybe"});
  const std::vector<Identifier> dependent{var2->Id()};
  const std::vector<Identifier> conditioning{var1->Id()};
  EXPECT_CALL(*constraint, HasDependentVariables()).WillRepeatedly(Return(true));
  EXPECT_CALL(*constraint, HasConditioningVariables()).WillRepeatedly(Return(true));
  EXPECT_CALL(*constraint, DependentVariables()).WillRepeatedly(Return(dependent));
  EXPECT_CALL(*constraint, ConditioningVariables()).WillRepeatedly(Return(conditioning));

  auto result = this->network_.MaybeAdd(constraint);
  ASSERT_EQ(result, constraint);
  ASSERT_EQ(this->network_.GetConstraint(constraint->Id()), constraint);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddExistingConstraint) {
  auto existing = this->constraints_[0];
  auto duplicate = std::make_shared<MockConstraint_<TypeParam>>(existing->Id());
  // A genuine duplicate covers the same variables as the stored constraint.
  const std::vector<Identifier> dependent{this->variables_[1]->Id()};
  const std::vector<Identifier> conditioning{this->variables_[0]->Id()};
  ON_CALL(*duplicate, HasDependentVariables()).WillByDefault(Return(true));
  ON_CALL(*duplicate, HasConditioningVariables()).WillByDefault(Return(true));
  ON_CALL(*duplicate, DependentVariables()).WillByDefault(Return(dependent));
  ON_CALL(*duplicate, ConditioningVariables()).WillByDefault(Return(conditioning));

  auto result = this->network_.MaybeAdd(duplicate);
  ASSERT_TRUE(result);
  auto resolved = Unwrap(result);
  EXPECT_EQ(resolved, existing);
  EXPECT_NE(resolved, duplicate);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddConstraintWithDifferentVariablesReturnsError) {
  auto existing = this->constraints_[0];
  auto mismatched = std::make_shared<MockConstraint_<TypeParam>>(existing->Id());
  // Same id, but the proposed constraint covers different variables.
  const std::vector<Identifier> dependent{this->variables_[4]->Id()};
  const std::vector<Identifier> conditioning{this->variables_[3]->Id()};
  ON_CALL(*mismatched, HasDependentVariables()).WillByDefault(Return(true));
  ON_CALL(*mismatched, HasConditioningVariables()).WillByDefault(Return(true));
  ON_CALL(*mismatched, DependentVariables()).WillByDefault(Return(dependent));
  ON_CALL(*mismatched, ConditioningVariables()).WillByDefault(Return(conditioning));

  auto result = this->network_.MaybeAdd(mismatched);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), Error::kFailure);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddConstraintWithMissingVariable) {
  auto constraint = std::make_shared<MockConstraint_<TypeParam>>(Identifier{"bad_constraint"});
  const std::vector<Identifier> conditioning{Identifier{"missing_var"}};
  EXPECT_CALL(*constraint, HasConditioningVariables()).WillOnce(Return(true));
  EXPECT_CALL(*constraint, ConditioningVariables()).WillOnce(Return(conditioning));

  ASSERT_THROW(this->network_.MaybeAdd(constraint), std::runtime_error);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddConstraintWithPartiallyValidVariablesLeavesNetworkUnchanged) {
  auto variable = std::make_shared<MockVariable_<TypeParam>>(Identifier{"partially_valid_var_maybeadd"});
  ASSERT_TRUE(this->network_.Add(variable));

  auto constraint = std::make_shared<MockConstraint_<TypeParam>>(Identifier{"partially_valid_constraint_maybeadd"});
  const std::vector<Identifier> conditioning_variables{variable->Id(), Identifier{"missing_var"}};
  ON_CALL(*constraint, HasConditioningVariables()).WillByDefault(Return(true));
  ON_CALL(*constraint, ConditioningVariables()).WillByDefault(Return(conditioning_variables));
  ON_CALL(*constraint, HasDependentVariables()).WillByDefault(Return(false));

  ASSERT_THROW(this->network_.MaybeAdd(constraint), std::runtime_error);

  EXPECT_EQ(this->network_.GetConstraint(constraint->Id()), nullptr);
  EXPECT_EQ(ranges::distance(this->network_.GetConditionedConstraints(variable->Id())), 0);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddNewValue) {
  auto value = std::make_shared<Value<int>>(Identifier{"new_val"}, 42);
  auto result = this->network_.MaybeAdd(value);
  ASSERT_EQ(result, value);
  ASSERT_EQ(this->network_.GetValue(value->Id()), value);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddExistingValue) {
  auto existing = this->values_[0];
  auto duplicate = std::make_shared<Value<int>>(existing->Id(), 999);
  auto result = this->network_.MaybeAdd(duplicate);
  ASSERT_EQ(result, existing);
  ASSERT_NE(result, duplicate);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddWithConstructor) {
  const auto var_id = Identifier{"constructed_var_maybe"};
  auto var1 = this->network_.template MaybeAdd<MockVariable_<TypeParam>>(var_id);
  ASSERT_TRUE(var1);
  auto resolved1 = Unwrap(var1);
  ASSERT_EQ(this->network_.GetVariable(var_id), resolved1);

  auto var2 = this->network_.template MaybeAdd<MockVariable_<TypeParam>>(var_id);
  ASSERT_TRUE(var2);
  auto resolved2 = Unwrap(var2);
  ASSERT_EQ(resolved1, resolved2);
}

TYPED_TEST(DynamicBayesianNetworkTest, MaybeAddValueWithConstructor) {
  const auto value_id = Identifier{"constructed_value_maybe"};
  auto value1 = this->network_.template MaybeAdd<Value<std::string>>(value_id, "hello");
  ASSERT_TRUE(value1);
  ASSERT_NE(Unwrap(value1), nullptr);
  auto typed_value = this->network_.template GetValue<std::string>(value_id);
  ASSERT_TRUE(typed_value);
  auto typed_value_data = Unwrap(typed_value);
  ASSERT_TRUE(typed_value_data.has_value());
  ASSERT_EQ(typed_value_data.value(), "hello");

  auto value2 = this->network_.template MaybeAdd<Value<std::string>>(value_id, "world");
  ASSERT_EQ(value1, value2);
  auto typed_value2 = this->network_.template GetValue<std::string>(value_id);
  ASSERT_TRUE(typed_value2);
  auto typed_value2_data = Unwrap(typed_value2);
  ASSERT_TRUE(typed_value2_data.has_value());
  ASSERT_EQ(typed_value2_data.value(), "hello");  // Should still be "hello", not "world"
}

TYPED_TEST(DynamicBayesianNetworkTest, AcceptVisitor) {
  MockVisitor_<TypeParam> visitor;

  std::vector<Identifier> visited;

  EXPECT_CALL(visitor, Visit(::testing::A<const Variable_<TypeParam>&>()))
      .Times(7)
      .WillRepeatedly([&visited](const Variable_<TypeParam>& variable) { visited.push_back(variable.Id()); });
  EXPECT_CALL(visitor, Visit(::testing::A<const Constraint_<TypeParam>&>()))
      .Times(5)
      .WillRepeatedly([&visited](const Constraint_<TypeParam>& constraint) { visited.push_back(constraint.Id()); });
  EXPECT_CALL(visitor, Visit(::testing::A<const AbstractValue&>()))
      .Times(4)
      .WillRepeatedly([&visited](const AbstractValue& value) { visited.push_back(value.Id()); });

  this->network_.Accept(visitor);

  auto get_index = [&visited](const Identifier& id) {
    auto it = std::find(visited.begin(), visited.end(), id);
    EXPECT_TRUE(it != visited.end()) << "Identifier not found in visited list.";
    return std::distance(visited.begin(), it);
  };

  for (const auto& constraint : this->constraints_) {
    for (const auto& conditioning_id : constraint->ConditioningVariables()) {
      EXPECT_LT(get_index(conditioning_id), get_index(constraint->Id()));
    }
    for (const auto& dependent_id : constraint->DependentVariables()) {
      EXPECT_LT(get_index(constraint->Id()), get_index(dependent_id));
    }
  }
}

// Test GetConditionedConstraints
TYPED_TEST(DynamicBayesianNetworkTest, GetConditionedConstraintsSingleVar) {
  // Constraint 0 is conditioned on variables_[0]
  auto constraints = this->network_.GetConditionedConstraints(this->variables_[0]->Id());
  ASSERT_EQ(ranges::distance(constraints), 1);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetConditionedConstraintsMultipleVars) {
  // Constraints 0-4 are conditioned on variables_[0]..variables_[4] pairwise
  // variables_[0] conditions constraint 0
  // variables_[1] conditions constraint 0 and 1
  // So variables_[0,1] should return constraints 0 and 1
  std::array<Identifier, 2> ids{this->variables_[0]->Id(), this->variables_[1]->Id()};
  auto constraints = this->network_.GetConditionedConstraints(ids) | ranges::to<std::vector>();
  // Should find at least constraints 0 and 1
  EXPECT_GE(ranges::distance(constraints), 2);
  // Both constraints 0 and 1 should be included
  auto ids_found = constraints | ranges::views::transform([](const auto& constraint) { return constraint->Id(); });
  auto it0 = ranges::find(ids_found, this->constraints_[0]->Id());
  auto it1 = ranges::find(ids_found, this->constraints_[1]->Id());
  EXPECT_NE(it0, ranges::end(ids_found));
  EXPECT_NE(it1, ranges::end(ids_found));
}

TYPED_TEST(DynamicBayesianNetworkTest, GetConditionedConstraintsDedup) {
  // Both variables_[0] and variables_[1] condition constraint 0
  // (variables_[0] -> constraint 0, variables_[1] -> constraint 0)
  // Requesting both should still return constraint 0 only once
  std::array<Identifier, 2> ids{this->variables_[0]->Id(), this->variables_[1]->Id()};
  auto constraints = this->network_.GetConditionedConstraints(ids);
  // Count occurrences of constraint 0
  const int count = ranges::count_if(
      constraints, [this](const auto& constraint) { return constraint->Id() == this->constraints_[0]->Id(); });
  EXPECT_EQ(count, 1);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetConditionedConstraintsEmpty) {
  auto constraints = this->network_.GetConditionedConstraints(std_future::span<const Identifier>{});
  ASSERT_EQ(ranges::distance(constraints), 0);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetConditionedConstraintsNotFound) {
  Identifier unknown{"nonexistent"};
  auto constraints = this->network_.GetConditionedConstraints(unknown);
  ASSERT_EQ(ranges::distance(constraints), 0);
}

// GetConditioningConstraints tests
TYPED_TEST(DynamicBayesianNetworkTest, GetConditioningConstraintsSingleVar) {
  // Constraint 0 has variables_[1] as dependent, so querying variables_[1]
  // should return constraint 0 via the incoming adjacency list.
  // Use a named (lvalue) array so the span data stays alive for the lazy pipeline.
  const std::array<Identifier, 1> ids{this->variables_[1]->Id()};
  auto constraints = this->network_.GetConditioningConstraints(ids);
  int count = 0;
  Identifier found_id;
  for (const auto& c : constraints) {
    ++count;
    found_id = c->Id();
  }
  ASSERT_EQ(count, 1);
  ASSERT_EQ(found_id, this->constraints_[0]->Id());
}

TYPED_TEST(DynamicBayesianNetworkTest, GetConditioningConstraintsMultipleVars) {
  // variables_[0] has no incoming constraints.
  // variables_[1] is dependent of constraint 0.
  // variables_[2] is dependent of constraint 1.
  // So querying variables_[1, 2] should return constraints 0 and 1.
  std::array<Identifier, 2> ids{this->variables_[1]->Id(), this->variables_[2]->Id()};
  auto constraints = this->network_.GetConditioningConstraints(ids);
  bool found0 = false, found1 = false;
  int count = 0;
  for (const auto& c : constraints) {
    ++count;
    if (c->Id() == this->constraints_[0]->Id())
      found0 = true;
    if (c->Id() == this->constraints_[1]->Id())
      found1 = true;
  }
  EXPECT_GE(count, 2);
  EXPECT_TRUE(found0);
  EXPECT_TRUE(found1);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetConditioningConstraintsDedup) {
  // If two dependent variables share the same constraint incoming, dedup works.
  // Set up: a constraint with two dependent variables would cause this,
  // but in our fixture each constraint has one dependent. Instead verify
  // that querying the same variable twice doesn't double-count.
  std::array<Identifier, 2> ids{this->variables_[1]->Id(), this->variables_[1]->Id()};
  auto constraints = this->network_.GetConditioningConstraints(ids);
  int count = 0;
  for (auto it = constraints.begin(); it != constraints.end(); ++it) {
    ++count;
  }
  EXPECT_EQ(count, 1);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetConditioningConstraintsEmpty) {
  auto constraints = this->network_.GetConditioningConstraints(std_future::span<const Identifier>{});
  ASSERT_EQ(ranges::distance(constraints), 0);
}

TYPED_TEST(DynamicBayesianNetworkTest, GetConditioningConstraintsNotFound) {
  Identifier unknown{"nonexistent"};
  auto constraints = this->network_.GetConditioningConstraints(unknown);
  ASSERT_EQ(ranges::distance(constraints), 0);
}

// HasConditioningConstraints tests
TYPED_TEST(DynamicBayesianNetworkTest, HasConditioningConstraints) {
  // variables_[1] is the dependent variable of constraint 0, so it is
  // conditioned by the network; variables_[0] only conditions constraint 0.
  EXPECT_TRUE(this->network_.HasConditioningConstraints(this->variables_[1]->Id()));
  EXPECT_FALSE(this->network_.HasConditioningConstraints(this->variables_[0]->Id()));
  EXPECT_FALSE(this->network_.HasConditioningConstraints(Identifier{"nonexistent"}));
}

// HasConditionedConstraints tests
TYPED_TEST(DynamicBayesianNetworkTest, HasConditionedConstraints) {
  // variables_[0] is a conditioning variable of constraint 0; variables_[1]
  // also conditions constraint 0 but is additionally a dependent variable.
  // Query a variable that is purely a conditioning variable.
  // variables_[6] participates in no constraint at all.
  EXPECT_TRUE(this->network_.HasConditionedConstraints(this->variables_[0]->Id()));
  EXPECT_FALSE(this->network_.HasConditionedConstraints(this->variables_[6]->Id()));
  EXPECT_FALSE(this->network_.HasConditionedConstraints(Identifier{"nonexistent"}));
}

TYPED_TEST(DynamicBayesianNetworkTest, HasConstraintsAfterRemove) {
  auto constraint = this->constraints_.front();
  ASSERT_TRUE(this->network_.HasConditioningConstraints(this->variables_[1]->Id()));
  ASSERT_TRUE(this->network_.HasConditionedConstraints(this->variables_[0]->Id()));

  ASSERT_TRUE(this->network_.Remove(constraint));

  EXPECT_FALSE(this->network_.HasConditioningConstraints(this->variables_[1]->Id()));
  EXPECT_FALSE(this->network_.HasConditionedConstraints(this->variables_[0]->Id()));
}

// GetEarliestVariable tests
TYPED_TEST(DynamicBayesianNetworkTest, GetEarliestVariable) {
  auto earliest = this->network_.GetEarliestVariable("var");
  ASSERT_NE(earliest, nullptr);
  ASSERT_EQ(earliest->Id(), this->variables_[0]->Id());
}

TYPED_TEST(DynamicBayesianNetworkTest, GetEarliestVariableNotFound) {
  auto earliest = this->network_.GetEarliestVariable("nonexistent");
  ASSERT_EQ(earliest, nullptr);
}

}  // namespace

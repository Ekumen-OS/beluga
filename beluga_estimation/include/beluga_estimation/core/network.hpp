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

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <range/v3/view/any_view.hpp>
#include <range/v3/view/cache1.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/type_casts.hpp>
#include <beluga_estimation/common/type_traits.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/constraint.hpp>
#include <beluga_estimation/core/value.hpp>
#include <beluga_estimation/core/variable.hpp>

namespace beluga_estimation {

template <typename T>
class DynamicBayesianNetworkVisitor_;

/// A Dynamic Bayesian Network (DBN) is a probabilistic graphical model
/// that represents dependencies among variables over time.
///
/// It is composed of nodes representing random variables and directed edges
/// representing conditional dependencies. The "dynamic" aspect refers to the
/// modeling of temporal sequences, where the state of the network at one
/// point in time can influence its state at the next.
///
/// @tparam T The underlying data type for the network, typically float or double.
template <typename T>
class DynamicBayesianNetwork_ final {
 public:
  static constexpr std::string_view kDefaultName = "any";

  /// @brief Default constructor.
  DynamicBayesianNetwork_();
  /// @brief Default destructor.
  ~DynamicBayesianNetwork_();

  /// @name Variable APIs
  /// @{

  /// @brief Get a variable by its identifier.
  /// @param id The identifier of the variable to retrieve.
  /// @return A shared pointer to the variable, or `nullptr` if not found.
  std::shared_ptr<const Variable_<T>> GetVariable(const Identifier& id) const;

  /// @brief Get a safely downcasted variable by its identifier.
  /// @tparam ConcreteVariableT The concrete variable type to cast to.
  /// @param id The identifier of the variable to retrieve.
  /// @return `Ok(nullptr)` if not found, `Ok(variable)` on success, or an error
  ///         with `Error::kFailure` if the stored variable has a different type.
  template <class ConcreteVariableT>
  auto GetVariable(const Identifier& id) const {
    return safe_dynamic_pointer_cast<const ConcreteVariableT>(GetVariable(id), "variable", id.name);
  }

  /// @brief Get a safely downcasted variable by its identifier.
  /// @tparam ConcreteVariable The concrete variable type to cast to.
  /// @param id The identifier of the variable to retrieve.
  /// @return `Ok(nullptr)` if not found, `Ok(variable)` on success, or an error
  ///         with `Error::kFailure` if the stored variable has a different type.
  template <template <typename...> class ConcreteVariable>
  auto GetVariable(const Identifier& id) const {
    return GetVariable<ConcreteVariable<T>>(id);
  }

  /// @brief Get the chronologically last variable with a given name.
  /// @param name The name of the variable to retrieve.
  /// @return A shared pointer to the latest variable with that name, or `nullptr` if none exists.
  std::shared_ptr<const Variable_<T>> GetLatestVariable(std::string_view name) const;

  /// @brief Get the chronologically last variable before or at a given time.
  /// @param time The time point to query.
  /// @return A shared pointer to the latest variable, or `nullptr` if none exists before or at that time.
  std::shared_ptr<const Variable_<T>> GetLatestVariable(TimePoint time) const;

  /// @brief Get the chronologically last variable with a given name before or at a given time.
  /// @param name The name of the variable to retrieve.
  /// @param time The time point to query.
  /// @return A shared pointer to the latest variable, or `nullptr` if none exists with that name
  ///   before or at that time.
  std::shared_ptr<const Variable_<T>> GetLatestVariable(std::string_view name, TimePoint time) const;

  /// @brief Get the latest, safely downcasted variable.
  /// @tparam ConcreteVariableT The concrete variable type to cast to.
  /// @param args The arguments to forward to the underlying `latest_variable` method.
  /// @return `Ok(nullptr)` if not found, `Ok(variable)` on success, or an error
  ///         with `Error::kFailure` if the stored variable has a different type.
  template <class ConcreteVariableT, typename... Args>
  auto GetLatestVariable(Args&&... args) const {
    auto variable = GetLatestVariable(std::forward<Args>(args)...);
    return safe_dynamic_pointer_cast<const ConcreteVariableT>(
        variable, "variable", variable != nullptr ? std::string_view{variable->Id().name} : std::string_view{});
  }

  /// @brief Get the latest, safely downcasted variable.
  /// @tparam ConcreteVariable The concrete variable type to cast to.
  /// @param args The arguments to forward to the underlying `latest_variable` method.
  /// @return `Ok(nullptr)` if not found, `Ok(variable)` on success, or an error
  ///         with `Error::kFailure` if the stored variable has a different type.
  template <template <typename...> class ConcreteVariable, typename... Args>
  auto GetLatestVariable(Args&&... args) const {
    return GetLatestVariable<ConcreteVariable<T>>(std::forward<Args>(args)...);
  }

  /// @brief Get the chronologically first variable with a given name.
  /// @param name The name of the variable to retrieve.
  /// @return A shared pointer to the earliest variable with that name, or `nullptr` if none exists.
  std::shared_ptr<const Variable_<T>> GetEarliestVariable(std::string_view name) const;

  /// @brief Get the earliest, safely downcasted variable.
  /// @tparam ConcreteVariableT The concrete variable type to cast to.
  /// @param name The name of the variable to retrieve.
  /// @return `Ok(nullptr)` if not found, `Ok(variable)` on success, or an error
  ///         with `Error::kFailure` if the stored variable has a different type.
  template <class ConcreteVariableT>
  auto GetEarliestVariable(std::string_view name) const {
    return safe_dynamic_pointer_cast<const ConcreteVariableT>(GetEarliestVariable(name), "variable", name);
  }

  /// @brief Get the earliest, safely downcasted variable.
  /// @tparam ConcreteVariable The concrete variable type to cast to.
  /// @param name The name of the variable to retrieve.
  /// @return `Ok(nullptr)` if not found, `Ok(variable)` on success, or an error
  ///         with `Error::kFailure` if the stored variable has a different type.
  template <template <typename...> class ConcreteVariable>
  auto GetEarliestVariable(std::string_view name) const {
    return GetEarliestVariable<ConcreteVariable<T>>(name);
  }

  /// @brief Get a range with all variables in the network.
  /// @return A range of shared pointers to the variables.
  single_pass_view<std::shared_ptr<const Variable_<T>>> GetVariables() const;

  /// @brief Get a range with all variables matching a name.
  /// @param name The name to filter variables by.
  /// @return A range of shared pointers to the variables.
  single_pass_view<std::shared_ptr<const Variable_<T>>> GetVariables(std::string_view name) const;

  /// @brief Get a range with all variables within a time interval.
  /// @param interval The time interval to filter variables by.
  /// @return A range of shared pointers to the variables.
  single_pass_view<std::shared_ptr<const Variable_<T>>> GetVariables(const TimeInterval& interval) const;

  /// @brief Get a range with all variables matching a name and within a time interval.
  /// @param name The name to filter variables by.
  /// @param interval The time interval to filter variables by.
  /// @return A range of shared pointers to the variables.
  single_pass_view<std::shared_ptr<const Variable_<T>>> GetVariables(
      std::string_view name,
      const TimeInterval& interval) const;

  /// @brief Get a range of safely downcasted variables.
  /// @tparam ConcreteVariableT The concrete variable type to cast to.
  /// @tparam Args The types of the arguments to forward to the underlying `variables` method.
  /// @param args The arguments to forward to the underlying `variables` method.
  /// @return A range of results, each `Ok(variable)` on success or an error
  ///         with `Error::kFailure` if the variable has a different type.
  template <class ConcreteVariableT, typename... Args>
  auto GetVariables(Args&&... args) const {
    return GetVariables(std::forward<Args>(args)...) | ranges::views::transform([](const auto& variable) {
             return safe_dynamic_pointer_cast<const ConcreteVariableT>(variable, "variable", variable->Id().name);
           }) |
           ranges::views::cache1;
  }

  /// @brief Get a range of safely downcasted variables.
  /// @tparam ConcreteVariable The concrete variable type to cast to.
  /// @tparam Args The types of the arguments to forward to the underlying `variables` method.
  /// @param args The arguments to forward to the underlying `variables` method.
  /// @return A range of results, each `Ok(variable)` on success or an error
  ///         with `Error::kFailure` if the variable has a different type.
  template <template <typename...> class ConcreteVariable, typename... Args>
  auto GetVariables(Args&&... args) const {
    return GetVariables<ConcreteVariable<T>>(std::forward<Args>(args)...);
  }

  /// @}

  /// @name Constraint APIs
  /// @{

  /// @brief Get a constraint by its identifier.
  /// @param id The identifier of the constraint to retrieve.
  /// @return A shared pointer to the constraint, or `nullptr` if not found.
  std::shared_ptr<const Constraint_<T>> GetConstraint(const Identifier& id) const;

  /// @brief Get a safely downcasted constraint by its identifier.
  /// @tparam ConcreteConstraintT The concrete constraint type to cast to.
  /// @tparam Args The types of the arguments to forward to the underlying `constraint` method.
  /// @param args The arguments to forward to the underlying `constraint` method.
  /// @return `Ok(nullptr)` if not found, `Ok(constraint)` on success, or an error
  ///         with `Error::kFailure` if the stored constraint has a different type.
  template <class ConcreteConstraintT, typename... Args>
  auto GetConstraint(const Identifier& id) const {
    return safe_dynamic_pointer_cast<const ConcreteConstraintT>(GetConstraint(id), "constraint", id.name);
  }

  /// @brief Get a safely downcasted constraint by its identifier.
  /// @tparam ConcreteConstraint The concrete constraint type to cast to.
  /// @tparam Args The types of the arguments to forward to the underlying `constraint` method.
  /// @param args The arguments to forward to the underlying `constraint` method.
  /// @return `Ok(nullptr)` if not found, `Ok(constraint)` on success, or an error
  ///         with `Error::kFailure` if the stored constraint has a different type.
  template <template <typename...> class ConcreteConstraint, typename... Args>
  auto GetConstraint(const Identifier& id) const {
    return GetConstraint<ConcreteConstraint<T>>(id);
  }

  /// @brief Get the chronologically last constraint with a given name.
  /// @param name The name of the constraint to retrieve.
  /// @return A shared pointer to the latest constraint with that name, or `nullptr` if none exists.
  std::shared_ptr<const Constraint_<T>> GetLatestConstraint(std::string_view name) const;

  /// @brief Get the chronologically last constraint before or at a given time.
  /// @param time The time point to query.
  /// @return A shared pointer to the latest constraint, or `nullptr` if none exists before or at that time.
  std::shared_ptr<const Constraint_<T>> GetLatestConstraint(TimePoint time) const;

  /// @brief Get the chronologically last constraint with a given name before or at a given time.
  /// @param name The name of the constraint to retrieve.
  /// @param time The time point to query.
  /// @return A shared pointer to the latest constraint, or `nullptr` if none exists with that name
  ///  before or at that time.
  std::shared_ptr<const Constraint_<T>> GetLatestConstraint(std::string_view name, TimePoint time) const;

  /// @brief Get the latest, safely downcasted constraint.
  /// @tparam ConcreteConstraintT The concrete constraint type to cast to.
  /// @tparam Args The types of the arguments to forward to the underlying `latest_constraint` method.
  /// @param args The arguments to forward to the underlying `latest_constraint` method.
  /// @return `Ok(nullptr)` if not found, `Ok(constraint)` on success, or an error
  ///         with `Error::kFailure` if the stored constraint has a different type.
  template <class ConcreteConstraintT, typename... Args>
  auto GetLatestConstraint(Args&&... args) const {
    auto constraint = GetLatestConstraint(std::forward<Args>(args)...);
    return safe_dynamic_pointer_cast<const ConcreteConstraintT>(
        constraint, "constraint", constraint != nullptr ? std::string_view{constraint->Id().name} : std::string_view{});
  }

  /// @brief Get the latest, safely downcasted constraint.
  /// @tparam ConcreteConstraint The concrete constraint type to cast to.
  /// @tparam Args The types of the arguments to forward to the underlying `latest_constraint` method.
  /// @param args The arguments to forward to the underlying `latest_constraint` method.
  /// @return `Ok(nullptr)` if not found, `Ok(constraint)` on success, or an error
  ///         with `Error::kFailure` if the stored constraint has a different type.
  template <template <typename...> class ConcreteConstraint, typename... Args>
  auto GetLatestConstraint(Args&&... args) const {
    return GetLatestConstraint<ConcreteConstraint<T>>(std::forward<Args>(args)...);
  }

  /// @brief Get constraints conditioned on any of the given variables.
  ///
  /// Returns constraints that have any of the given identifiers as conditioning
  /// variables. Duplicate constraints (conditioned on multiple input variables)
  /// are returned only once.
  ///
  /// @param variable_ids The identifiers of the conditioning variables to query.
  /// @return A single-pass view of shared pointers to matching constraints.
  single_pass_view<std::shared_ptr<const Constraint_<T>>> GetConditionedConstraints(
      std_future::span<const Identifier> variable_ids) const;

  /// @brief Get constraints conditioned on a single variable.
  /// @param variable_id The identifier of the variable.
  /// @return A single-pass view of shared pointers to matching constraints.
  single_pass_view<std::shared_ptr<const Constraint_<T>>> GetConditionedConstraints(
      const Identifier& variable_id) const;

  /// @brief Get constraints conditioning any of the given variables.
  ///
  /// Returns constraints where any of the given identifiers appear as dependent
  /// variables (incoming edges). Duplicate constraints are returned only once.
  ///
  /// @param variable_ids The identifiers of the dependent variables to query.
  /// @return A single-pass view of shared pointers to matching constraints.
  single_pass_view<std::shared_ptr<const Constraint_<T>>> GetConditioningConstraints(
      std_future::span<const Identifier> variable_ids) const;

  /// @brief Get constraints conditioning a single variable.
  /// @param variable_id The identifier of the variable.
  /// @return A single-pass view of shared pointers to matching constraints.
  single_pass_view<std::shared_ptr<const Constraint_<T>>> GetConditioningConstraints(
      const Identifier& variable_id) const;

  /// @brief Check whether any constraint conditions a given variable.
  ///
  /// Equivalent to checking that @ref GetConditioningConstraints returns a
  /// non-empty range, but it performs a single adjacency map lookup instead of
  /// materializing constraint references, making it suitable for hot paths that
  /// only need to know whether a variable is already modeled.
  ///
  /// @param variable_id The identifier of the variable to query.
  /// @return `true` if at least one constraint has @p variable_id as a
  ///   dependent variable (i.e. it conditions it), `false` otherwise.
  bool HasConditioningConstraints(const Identifier& variable_id) const;

  /// @brief Check whether any constraint is conditioned on a given variable.
  ///
  /// Equivalent to checking that @ref GetConditionedConstraints returns a
  /// non-empty range, but it performs a single adjacency map lookup instead of
  /// materializing constraint references.
  ///
  /// @param variable_id The identifier of the variable to query.
  /// @return `true` if at least one constraint has @p variable_id as a
  ///   conditioning variable, `false` otherwise.
  bool HasConditionedConstraints(const Identifier& variable_id) const;

  /// @brief Get a range with all constraints in the network.
  /// @return A range of shared pointers to the constraints.
  single_pass_view<std::shared_ptr<const Constraint_<T>>> GetConstraints() const;

  /// @brief Get a range with all constraints matching a name.
  /// @param name The name to filter constraints by.
  /// @return A range of shared pointers to the constraints.
  single_pass_view<std::shared_ptr<const Constraint_<T>>> GetConstraints(std::string_view name) const;

  /// @brief Get a range with all constraints within a time interval.
  /// @param interval The time interval to filter constraints by.
  /// @return A range of shared pointers to the constraints.
  single_pass_view<std::shared_ptr<const Constraint_<T>>> GetConstraints(const TimeInterval& interval) const;

  /// @brief Get a range with all constraints matching a name and within a time interval.
  /// @param name The name to filter constraints by.
  /// @param interval The time interval to filter constraints by.
  /// @return A range of shared pointers to the constraints.
  single_pass_view<std::shared_ptr<const Constraint_<T>>> GetConstraints(
      std::string_view name,
      const TimeInterval& interval) const;

  /// @brief Get a range of safely downcasted constraints.
  /// @tparam ConcreteConstraintT The concrete constraint type to cast to.
  /// @tparam Args The types of the arguments to forward to the underlying `constraints` method.
  /// @param args The arguments to forward to the underlying `constraints` method.
  /// @return A range of results, each `Ok(constraint)` on success or an error
  ///         with `Error::kFailure` if the constraint has a different type.
  template <class ConcreteConstraintT, typename... Args>
  auto GetConstraints(Args&&... args) const {
    return GetConstraints(std::forward<Args>(args)...) | ranges::views::transform([](const auto& constraint) {
             return safe_dynamic_pointer_cast<const ConcreteConstraintT>(
                 constraint, "constraint", constraint->Id().name);
           }) |
           ranges::views::cache1;
  }

  /// @brief Get a range of safely downcasted constraints.
  /// @tparam ConcreteConstraint The concrete constraint type to cast to.
  /// @tparam Args The types of the arguments to forward to the underlying `constraints` method.
  /// @param args The arguments to forward to the underlying `constraints` method.
  /// @return A range of results, each `Ok(constraint)` on success or an error
  ///         with `Error::kFailure` if the constraint has a different type.
  template <template <typename...> class ConcreteConstraint, typename... Args>
  auto GetConstraints(Args&&... args) const {
    return GetConstraints<ConcreteConstraint<T>>(std::forward<Args>(args)...);
  }

  /// @}

  /// @name Value APIs
  /// @{

  /// @brief Get a value by its identifier.
  /// @param id The identifier of the value to retrieve.
  /// @return A shared pointer to the value, or `nullptr` if not found.
  std::shared_ptr<const AbstractValue> GetValue(const Identifier& id) const;

  /// @brief Get the data of a typed value.
  /// @tparam U The data type of the value.
  /// @param id The identifier of the value to retrieve.
  /// @return `Ok(nullopt)` if not found, `Ok(data)` on success, or an error
  ///         with `Error::kFailure` if the stored value has a different type.
  template <class U>
  Result_<nullable_t<U>> GetValue(const Identifier& id) const {
    auto result = safe_dynamic_pointer_cast<const Value<U>>(GetValue(id), "value", id.name);
    if (!result) {
      return Err(CollectError(result));
    }
    auto typed_value = Unwrap(result);
    return typed_value ? nullable_t<U>{typed_value->GetData()} : nullable_t<U>{};
  }

  /// @brief Get the chronologically last value with a given name.
  /// @param name The name of the value to retrieve.
  /// @return A shared pointer to the latest value with that name, or `nullptr` if none exists.
  std::shared_ptr<const AbstractValue> GetLatestValue(std::string_view name) const;

  /// @brief Get the chronologically last value before or at a given time.
  /// @param time The time point to query.
  /// @return A shared pointer to the latest value, or `nullptr` if none exists before or at that time.
  std::shared_ptr<const AbstractValue> GetLatestValue(TimePoint time) const;

  /// @brief Get the chronologically last value with a given name before or at a given time.
  /// @param name The name of the value to retrieve.
  /// @param time The time point to query.
  /// @return A shared pointer to the latest value, or `nullptr` if none exists with that name
  ///   before or at that time.
  std::shared_ptr<const AbstractValue> GetLatestValue(std::string_view name, TimePoint time) const;

  /// @brief Get the latest typed value's data.
  /// @tparam U The data type of the value.
  /// @param args The arguments to forward to the underlying `latest_value` method.
  /// @return `Ok(nullopt)` if not found, `Ok(data)` on success, or an error
  ///         with `Error::kFailure` if the stored value has a different type.
  template <class U, typename... Args>
  Result_<nullable_t<U>> GetLatestValue(Args&&... args) const {
    auto value = GetLatestValue(std::forward<Args>(args)...);
    auto result = safe_dynamic_pointer_cast<const Value<U>>(
        value, "value", value != nullptr ? std::string_view{value->Id().name} : std::string_view{});
    if (!result) {
      return Err(CollectError(result));
    }
    auto typed_value = Unwrap(result);
    return typed_value ? nullable_t<U>{typed_value->GetData()} : nullable_t<U>{};
  }

  /// @brief Get a range with all values in the network.
  /// @return A range of shared pointers to the values.
  single_pass_view<std::shared_ptr<const AbstractValue>> GetValues() const;

  /// @brief Get a range with all values matching a name.
  /// @param name The name to filter values by.
  /// @return A range of shared pointers to the values.
  single_pass_view<std::shared_ptr<const AbstractValue>> GetValues(std::string_view name) const;

  /// @brief Get a range with all values within a time interval.
  /// @param interval The time interval to filter values by.
  /// @return A range of shared pointers to the values.
  single_pass_view<std::shared_ptr<const AbstractValue>> GetValues(const TimeInterval& interval) const;

  /// @brief Get a range with all values matching a name and within a time interval.
  /// @param name The name to filter values by.
  /// @param interval The time interval to filter values by.
  /// @return A range of shared pointers to the values.
  single_pass_view<std::shared_ptr<const AbstractValue>> GetValues(std::string_view name, const TimeInterval& interval)
      const;

  /// @brief Get a range of typed value's data.
  /// @tparam U The data type of the values.
  /// @param args The arguments to forward to the underlying `values` method.
  /// @return A lazy range of results holding each value's data, or an error
  ///         with `Error::kFailure` for a value of a different type.
  template <class U, typename... Args>
  auto GetValues(Args&&... args) const {
    return GetValues(std::forward<Args>(args)...) | ranges::views::transform([](const auto& value) -> Result_<U> {
             auto result = safe_dynamic_pointer_cast<const Value<U>>(value, "value", value->Id().name);
             if (!result) {
               return Err(CollectError(result));
             }
             return Unwrap(result)->GetData();
           }) |
           ranges::views::cache1;
  }

  /// @}

  /// @name Modification APIs
  /// @{

  /// @brief Add a variable to the network.
  /// @param variable The variable to add.
  /// @return Success, or an error with `Error::kFailure` if the variable
  ///         already exists.
  Result Add(std::shared_ptr<const Variable_<T>> variable);

  /// @brief Add a constraint to the network.
  /// @param constraint The constraint to add.
  /// @return Success, or an error with `Error::kFailure` if the constraint
  ///         already exists.
  /// @pre Every variable referenced by the constraint already exists in the
  ///      network. This is checked with `BELUGA_ESTIMATION_DEMAND` before the network is
  ///      modified, so a violation leaves the network unchanged.
  Result Add(std::shared_ptr<const Constraint_<T>> constraint);

  /// @brief Add a value to the network.
  /// @param value The value to add.
  /// @return Success, or an error with `Error::kFailure` if the value already
  ///         exists.
  Result Add(std::shared_ptr<const AbstractValue> value);

  /// @brief Add a variable to the network if it doesn't exist.
  /// @param variable The variable to add.
  /// @return The variable in the network (either newly added or pre-existing),
  ///         or an error with `Error::kFailure` if a variable with the same id
  ///         but a different type is already present.
  Result_<std::shared_ptr<const Variable_<T>>> MaybeAdd(std::shared_ptr<const Variable_<T>> variable);

  /// @brief Add a constraint to the network if it doesn't exist.
  /// @param constraint The constraint to add.
  /// @return The constraint in the network (either newly added or pre-existing),
  ///         or an error with `Error::kFailure` if a constraint with the same id
  ///         but different variables is already present.
  /// @pre If the constraint is not already present, every variable it
  ///      references must already exist. This is checked with `BELUGA_ESTIMATION_DEMAND`
  ///      before the network is modified.
  Result_<std::shared_ptr<const Constraint_<T>>> MaybeAdd(std::shared_ptr<const Constraint_<T>> constraint);

  /// @brief Add a value to the network if it doesn't exist.
  /// @param value The value to add.
  /// @return The value in the network (either newly added or pre-existing), or
  ///         an error with `Error::kFailure` if a value with the same id but a
  ///         different type is already present.
  Result_<std::shared_ptr<const AbstractValue>> MaybeAdd(std::shared_ptr<const AbstractValue> value);

  /// @brief The stored element type for a given element or value type @p U.
  template <class U>
  using ElementType = std::conditional_t<
      std::is_base_of_v<Variable_<T>, U> || std::is_base_of_v<Constraint_<T>, U> || std::is_base_of_v<AbstractValue, U>,
      U,
      Value<U>>;

  /// @brief Construct and add an element to the network if it doesn't exist.
  /// @tparam U The type of element to add. Can be a Variable, Constraint, AbstractValue, or other data type.
  /// @tparam Args The types of the arguments to forward to the element's constructor.
  /// @param args The arguments to forward to the element's constructor.
  /// @return The element in the network (either newly constructed or pre-existing),
  ///         or an error with `Error::kFailure` if an element with the same id
  ///         but a different type or structure is already present.
  template <class U, typename... Args>
  Result_<std::shared_ptr<const ElementType<U>>> MaybeAdd(Args&&... args) {
    using ElementT = ElementType<U>;
    auto element = std::make_shared<std::add_const_t<ElementT>>(std::forward<Args>(args)...);
    auto result = MaybeAdd(element);
    if (!result) {
      return Err(CollectError(result));
    }
    auto typed = std::dynamic_pointer_cast<const ElementT>(Unwrap(result));
    if (typed == nullptr) {
      return Err(
          Error::kFailure,
          fmt::format("element with id '{}' already exists with a different type", element->Id().name));
    }
    return typed;
  }

  /// @brief Construct and add an element to the network if it doesn't exist.
  /// @tparam Element The template template for the element to add.
  /// @tparam Args The types of the arguments to forward to the element's constructor.
  /// @param args The arguments to forward to the element's constructor.
  /// @return The element in the network (either newly constructed or pre-existing).
  template <template <typename...> class Element, typename... Args>
  auto MaybeAdd(Args&&... args) {
    return MaybeAdd<Element<T>>(std::forward<Args>(args)...);
  }

  /// @brief Construct and add an element to the network.
  /// @tparam U The type of element to add. Can be a Variable, Constraint, AbstractValue, or other data type.
  /// @tparam Args The types of the arguments to forward to the element's constructor.
  /// @param args The arguments to forward to the element's constructor.
  /// @return The newly created element, or an error with `Error::kFailure` if an
  ///         element with the same id already exists.
  template <class U, typename... Args>
  Result_<std::shared_ptr<const ElementType<U>>> Add(Args&&... args) {
    using ElementT = ElementType<U>;
    auto element = std::make_shared<std::add_const_t<ElementT>>(std::forward<Args>(args)...);
    auto result = Add(element);
    if (!result) {
      return Err(CollectError(result));
    }
    return element;
  }

  /// @brief Construct and add an element to the network.
  /// @tparam Element The template template for the element to add.
  /// @tparam Args The types of the arguments to forward to the element's constructor.
  /// @param args The arguments to forward to the element's constructor.
  /// @return A shared pointer to the newly created element, or `nullptr`
  /// if an element with the same ID already exists.
  template <template <typename...> class Element, typename... Args>
  auto Add(Args&&... args) {
    return Add<Element<T>>(std::forward<Args>(args)...);
  }

  /// @brief Update a variable in the network.
  /// @param variable The variable to update.
  /// @return Success, or an error with `Error::kFailure` if the variable does
  ///         not exist.
  Result Update(std::shared_ptr<const Variable_<T>> variable);

  /// @brief Update a constraint in the network.
  /// @param constraint The constraint to update.
  /// @return Success, or an error with `Error::kFailure` if the constraint does
  ///         not exist.
  /// @pre The new constraint must be equivalent to the stored one, i.e. it must
  ///      have the same dependent and conditioning variables. This is checked
  ///      with `BELUGA_ESTIMATION_DEMAND` before the network is modified.
  Result Update(std::shared_ptr<const Constraint_<T>> constraint);

  /// @brief Update a value in the network.
  /// @param value The value to update.
  /// @return Success, or an error with `Error::kFailure` if the value does not
  ///         exist.
  Result Update(std::shared_ptr<const AbstractValue> value);

  /// @brief Remove a variable from the network.
  /// @param variable The variable to remove.
  /// @return Success, or an error with `Error::kFailure` if the variable does
  ///         not exist.
  /// @pre The variable must not be connected to any constraint. This is checked
  ///      with `BELUGA_ESTIMATION_DEMAND` before the network is modified.
  Result Remove(std::shared_ptr<const Variable_<T>> variable);

  /// @brief Remove a constraint from the network.
  /// @param constraint The constraint to remove.
  /// @return Success, or an error with `Error::kFailure` if the constraint does
  ///         not exist.
  Result Remove(std::shared_ptr<const Constraint_<T>> constraint);

  /// @brief Remove a value from the network.
  /// @param value The value to remove.
  /// @return Success, or an error with `Error::kFailure` if the value does not
  ///         exist.
  Result Remove(std::shared_ptr<const AbstractValue> value);

  /// @brief Remove all elements from the network.
  void Clear();

  /// @}

  /// @name Visitor APIs
  /// @{

  /// @brief Accept a visitor to traverse the network.
  /// @param visitor The visitor to accept.
  /// @return `true` if a cycle was detected, `false` otherwise.
  bool Accept(DynamicBayesianNetworkVisitor_<T>& visitor) const;

  /// @}

 private:
  class Implementation;
  std::unique_ptr<Implementation> impl_;
};

using DynamicBayesianNetwork = DynamicBayesianNetwork_<DefaultScalar>;

}  // namespace beluga_estimation

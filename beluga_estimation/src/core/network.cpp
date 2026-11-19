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
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <fmt/format.h>

#include <range/v3/algorithm/equal.hpp>
#include <range/v3/view/empty.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/for_each.hpp>
#include <range/v3/view/map.hpp>
#include <range/v3/view/subrange.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/core/network_visitor.hpp>

namespace beluga_estimation {

namespace {

// Orders Identifier by (timestamp, name) with transparent TimePoint lookup.
//
// Used by DBN indices and the topological-sort ready queue where chronological
// traversal is the primary access pattern. Entries are clustered by timestamp
// first, so all elements at the same time point are contiguous, and
// lower_bound/upper_bound with a TimePoint argument finds the correct
// chronological position regardless of name.
struct TimeAwareIdentifierCompare {
  using is_transparent = void;

  bool operator()(const Identifier& a, const Identifier& b) const {
    return std::tie(a.timestamp, a.name) < std::tie(b.timestamp, b.name);
  }

  bool operator()(const Identifier& a, const TimePoint& b) const { return a.timestamp < b; }

  bool operator()(const TimePoint& a, const Identifier& b) const { return a < b.timestamp; }
};

// @brief Computes the index range covered by a time interval.
//
// @tparam Index A time-aware index over identifiers (e.g.,
//   `std::set<Identifier, TimeAwareIdentifierCompare>`).
// @param index The index to query.
// @param interval The time interval to map.
// @return The half-open iterator range `[first, last)` covered by @p interval.
template <typename Index>
auto make_index_range(const Index& index, const TimeInterval& interval) {
  auto first = interval.lower_bound.has_value()
                   ? (interval.lower_bound->closed ? index.lower_bound(interval.lower_bound->value)
                                                   : index.upper_bound(interval.lower_bound->value))
                   : index.begin();
  auto last = interval.upper_bound.has_value()
                  ? (interval.upper_bound->closed ? index.upper_bound(interval.upper_bound->value)
                                                  : index.lower_bound(interval.upper_bound->value))
                  : index.end();
  return std::make_pair(first, last);
}

}  // namespace

template <typename T>
class DynamicBayesianNetwork_<T>::Implementation {
 public:
  std::unordered_map<Identifier, std::vector<Identifier>> incoming_adjacency_list_;
  std::unordered_map<Identifier, std::vector<Identifier>> outgoing_adjacency_list_;

  std::unordered_map<Identifier, std::shared_ptr<const Variable_<T>>> variables_;
  std::unordered_map<Identifier, std::shared_ptr<const Constraint_<T>>> constraints_;
  std::unordered_map<Identifier, std::shared_ptr<const AbstractValue>> values_;

  std::unordered_map<std::string, std::set<Identifier, TimeAwareIdentifierCompare>> variable_indices_;
  std::unordered_map<std::string, std::set<Identifier, TimeAwareIdentifierCompare>> constraint_indices_;
  std::unordered_map<std::string, std::set<Identifier, TimeAwareIdentifierCompare>> value_indices_;
};

template <typename T>
DynamicBayesianNetwork_<T>::DynamicBayesianNetwork_() {
  impl_ = std::make_unique<Implementation>();
}

template <typename T>
DynamicBayesianNetwork_<T>::~DynamicBayesianNetwork_() = default;

// Variable implementations
template <typename T>
std::shared_ptr<const Variable_<T>> DynamicBayesianNetwork_<T>::GetVariable(const Identifier& id) const {
  auto it = impl_->variables_.find(id);
  return it != impl_->variables_.end() ? it->second : nullptr;
}

template <typename T>
std::shared_ptr<const Variable_<T>> DynamicBayesianNetwork_<T>::GetLatestVariable(std::string_view name) const {
  auto index_it = impl_->variable_indices_.find(std::string{name});
  if (index_it == impl_->variable_indices_.end() || index_it->second.empty()) {
    return nullptr;
  }
  auto var_it = impl_->variables_.find(*index_it->second.rbegin());
  return var_it != impl_->variables_.end() ? var_it->second : nullptr;
}

template <typename T>
std::shared_ptr<const Variable_<T>> DynamicBayesianNetwork_<T>::GetLatestVariable(TimePoint time) const {
  return GetLatestVariable(DynamicBayesianNetwork_<T>::kDefaultName, time);
}

template <typename T>
std::shared_ptr<const Variable_<T>> DynamicBayesianNetwork_<T>::GetLatestVariable(std::string_view name, TimePoint time)
    const {
  auto index_it = impl_->variable_indices_.find(std::string{name});
  if (index_it == impl_->variable_indices_.end()) {
    return nullptr;
  }
  const auto& index = index_it->second;
  auto it = index.upper_bound(time);
  if (it == index.begin()) {
    return nullptr;
  }
  auto var_it = impl_->variables_.find(*std::prev(it));
  return var_it != impl_->variables_.end() ? var_it->second : nullptr;
}

template <typename T>
std::shared_ptr<const Variable_<T>> DynamicBayesianNetwork_<T>::GetEarliestVariable(std::string_view name) const {
  auto index_it = impl_->variable_indices_.find(std::string{name});
  if (index_it == impl_->variable_indices_.end() || index_it->second.empty()) {
    return nullptr;
  }
  auto var_it = impl_->variables_.find(*index_it->second.begin());
  return var_it != impl_->variables_.end() ? var_it->second : nullptr;
}

template <typename T>
single_pass_view<std::shared_ptr<const Variable_<T>>> DynamicBayesianNetwork_<T>::GetVariables() const {
  return impl_->variables_ | ranges::views::values;
}

template <typename T>
single_pass_view<std::shared_ptr<const Variable_<T>>> DynamicBayesianNetwork_<T>::GetVariables(
    std::string_view name) const {
  auto index_it = impl_->variable_indices_.find(std::string{name});
  if (index_it == impl_->variable_indices_.end()) {
    return {};
  }
  const auto& index = index_it->second;
  return index | ranges::views::transform([this](const Identifier& id) { return impl_->variables_.at(id); });
}

template <typename T>
single_pass_view<std::shared_ptr<const Variable_<T>>> DynamicBayesianNetwork_<T>::GetVariables(
    const TimeInterval& interval) const {
  return GetVariables(DynamicBayesianNetwork_<T>::kDefaultName, interval);
}

template <typename T>
single_pass_view<std::shared_ptr<const Variable_<T>>> DynamicBayesianNetwork_<T>::GetVariables(
    std::string_view name,
    const TimeInterval& interval) const {
  auto index_it = impl_->variable_indices_.find(std::string{name});
  if (index_it == impl_->variable_indices_.end()) {
    return {};
  }
  const auto& index = index_it->second;
  auto [first, last] = make_index_range(index, interval);
  return ranges::make_subrange(first, last) |
         ranges::views::transform([this](const Identifier& id) { return impl_->variables_.at(id); });
}

// Constraint implementations
template <typename T>
std::shared_ptr<const Constraint_<T>> DynamicBayesianNetwork_<T>::GetConstraint(const Identifier& id) const {
  auto it = impl_->constraints_.find(id);
  return it != impl_->constraints_.end() ? it->second : nullptr;
}

template <typename T>
std::shared_ptr<const Constraint_<T>> DynamicBayesianNetwork_<T>::GetLatestConstraint(std::string_view name) const {
  auto index_it = impl_->constraint_indices_.find(std::string{name});
  if (index_it == impl_->constraint_indices_.end() || index_it->second.empty()) {
    return nullptr;
  }
  const auto& index = index_it->second;
  auto constraint_it = impl_->constraints_.find(*index.rbegin());
  return constraint_it != impl_->constraints_.end() ? constraint_it->second : nullptr;
}

template <typename T>
std::shared_ptr<const Constraint_<T>> DynamicBayesianNetwork_<T>::GetLatestConstraint(TimePoint time) const {
  return GetLatestConstraint(DynamicBayesianNetwork_<T>::kDefaultName, time);
}

template <typename T>
std::shared_ptr<const Constraint_<T>> DynamicBayesianNetwork_<T>::GetLatestConstraint(
    std::string_view name,
    TimePoint time) const {
  auto index_it = impl_->constraint_indices_.find(std::string{name});
  if (index_it == impl_->constraint_indices_.end()) {
    return nullptr;
  }
  const auto& index = index_it->second;
  auto it = index.upper_bound(time);
  if (it == index.begin()) {
    return nullptr;
  }
  auto constraint_it = impl_->constraints_.find(*std::prev(it));
  return constraint_it != impl_->constraints_.end() ? constraint_it->second : nullptr;
}

template <typename T>
single_pass_view<std::shared_ptr<const Constraint_<T>>> DynamicBayesianNetwork_<T>::GetConstraints() const {
  return impl_->constraints_ | ranges::views::values;
}

template <typename T>
single_pass_view<std::shared_ptr<const Constraint_<T>>> DynamicBayesianNetwork_<T>::GetConstraints(
    std::string_view name) const {
  auto index_it = impl_->constraint_indices_.find(std::string{name});
  if (index_it == impl_->constraint_indices_.end()) {
    return {};
  }
  return index_it->second |
         ranges::views::transform([this](const Identifier& id) { return impl_->constraints_.at(id); });
}

template <typename T>
single_pass_view<std::shared_ptr<const Constraint_<T>>> DynamicBayesianNetwork_<T>::GetConstraints(
    const TimeInterval& interval) const {
  return GetConstraints(DynamicBayesianNetwork_<T>::kDefaultName, interval);
}

template <typename T>
single_pass_view<std::shared_ptr<const Constraint_<T>>> DynamicBayesianNetwork_<T>::GetConstraints(
    std::string_view name,
    const TimeInterval& interval) const {
  auto index_it = impl_->constraint_indices_.find(std::string{name});
  if (index_it == impl_->constraint_indices_.end()) {
    return {};
  }
  const auto& index = index_it->second;
  auto [first, last] = make_index_range(index, interval);
  return ranges::make_subrange(first, last) |
         ranges::views::transform([this](const Identifier& id) { return impl_->constraints_.at(id); });
}

template <typename T>
single_pass_view<std::shared_ptr<const Constraint_<T>>> DynamicBayesianNetwork_<T>::GetConditionedConstraints(
    std_future::span<const Identifier> variable_ids) const {
  return ranges::views::for_each(
      variable_ids, [this, seen = std::unordered_set<Identifier>{}](const Identifier& variable_id) mutable {
        return ranges::yield_from(
            GetConditionedConstraints(variable_id) |
            ranges::views::filter([&seen](const auto& constraint) { return seen.insert(constraint->Id()).second; }));
      });
}

template <typename T>
single_pass_view<std::shared_ptr<const Constraint_<T>>> DynamicBayesianNetwork_<T>::GetConditioningConstraints(
    std_future::span<const Identifier> variable_ids) const {
  return ranges::views::for_each(
      variable_ids, [this, seen = std::unordered_set<Identifier>{}](const Identifier& variable_id) mutable {
        return ranges::yield_from(
            GetConditioningConstraints(variable_id) |
            ranges::views::filter([&seen](const auto& constraint) { return seen.insert(constraint->Id()).second; }));
      });
}

template <typename T>
single_pass_view<std::shared_ptr<const Constraint_<T>>> DynamicBayesianNetwork_<T>::GetConditionedConstraints(
    const Identifier& variable_id) const {
  auto it = impl_->outgoing_adjacency_list_.find(variable_id);
  if (it == impl_->outgoing_adjacency_list_.end()) {
    return {};
  }
  return it->second | ranges::views::transform([this](const Identifier& cid) { return impl_->constraints_.at(cid); });
}

template <typename T>
single_pass_view<std::shared_ptr<const Constraint_<T>>> DynamicBayesianNetwork_<T>::GetConditioningConstraints(
    const Identifier& variable_id) const {
  auto it = impl_->incoming_adjacency_list_.find(variable_id);
  if (it == impl_->incoming_adjacency_list_.end()) {
    return {};
  }
  return it->second | ranges::views::transform([this](const Identifier& cid) { return impl_->constraints_.at(cid); });
}

template <typename T>
bool DynamicBayesianNetwork_<T>::HasConditioningConstraints(const Identifier& variable_id) const {
  auto it = impl_->incoming_adjacency_list_.find(variable_id);
  return it != impl_->incoming_adjacency_list_.end() && !it->second.empty();
}

template <typename T>
bool DynamicBayesianNetwork_<T>::HasConditionedConstraints(const Identifier& variable_id) const {
  auto it = impl_->outgoing_adjacency_list_.find(variable_id);
  return it != impl_->outgoing_adjacency_list_.end() && !it->second.empty();
}

// Value implementations
template <typename T>
std::shared_ptr<const AbstractValue> DynamicBayesianNetwork_<T>::GetValue(const Identifier& id) const {
  auto it = impl_->values_.find(id);
  return it != impl_->values_.end() ? it->second : nullptr;
}

template <typename T>
std::shared_ptr<const AbstractValue> DynamicBayesianNetwork_<T>::GetLatestValue(std::string_view name) const {
  auto index_it = impl_->value_indices_.find(std::string{name});
  if (index_it == impl_->value_indices_.end() || index_it->second.empty()) {
    return nullptr;
  }
  auto value_it = impl_->values_.find(*index_it->second.rbegin());
  return value_it != impl_->values_.end() ? value_it->second : nullptr;
}

template <typename T>
std::shared_ptr<const AbstractValue> DynamicBayesianNetwork_<T>::GetLatestValue(TimePoint time) const {
  return GetLatestValue(DynamicBayesianNetwork_<T>::kDefaultName, time);
}

template <typename T>
std::shared_ptr<const AbstractValue> DynamicBayesianNetwork_<T>::GetLatestValue(std::string_view name, TimePoint time)
    const {
  auto index_it = impl_->value_indices_.find(std::string{name});
  if (index_it == impl_->value_indices_.end()) {
    return nullptr;
  }
  const auto& index = index_it->second;
  auto it = index.upper_bound(time);
  if (it == index.begin()) {
    return nullptr;
  }
  auto value_it = impl_->values_.find(*std::prev(it));
  return value_it != impl_->values_.end() ? value_it->second : nullptr;
}

template <typename T>
single_pass_view<std::shared_ptr<const AbstractValue>> DynamicBayesianNetwork_<T>::GetValues() const {
  return impl_->values_ | ranges::views::values;
}

template <typename T>
single_pass_view<std::shared_ptr<const AbstractValue>> DynamicBayesianNetwork_<T>::GetValues(
    std::string_view name) const {
  auto index_it = impl_->value_indices_.find(std::string{name});
  if (index_it == impl_->value_indices_.end()) {
    return {};
  }
  return index_it->second | ranges::views::transform([this](const Identifier& id) { return impl_->values_.at(id); });
}

template <typename T>
single_pass_view<std::shared_ptr<const AbstractValue>> DynamicBayesianNetwork_<T>::GetValues(
    const TimeInterval& interval) const {
  return GetValues(DynamicBayesianNetwork_<T>::kDefaultName, interval);
}

template <typename T>
single_pass_view<std::shared_ptr<const AbstractValue>> DynamicBayesianNetwork_<T>::GetValues(
    std::string_view name,
    const TimeInterval& interval) const {
  auto index_it = impl_->value_indices_.find(std::string{name});
  if (index_it == impl_->value_indices_.end()) {
    return {};
  }
  const auto& index = index_it->second;
  auto [first, last] = make_index_range(index, interval);
  return ranges::make_subrange(first, last) |
         ranges::views::transform([this](const Identifier& id) { return impl_->values_.at(id); });
}

template <typename T>
Result DynamicBayesianNetwork_<T>::Add(std::shared_ptr<const Variable_<T>> variable) {
  const auto& id = variable->Id();
  if (impl_->variables_.count(id) > 0) {
    return Err(Error::kFailure, fmt::format("variable '{}' already exists in the network", id.name));
  }
  impl_->incoming_adjacency_list_.emplace(id, std::vector<Identifier>{});
  impl_->outgoing_adjacency_list_.emplace(id, std::vector<Identifier>{});
  impl_->variable_indices_[std::string{DynamicBayesianNetwork_<T>::kDefaultName}].insert(id);
  impl_->variable_indices_[id.name].insert(id);
  impl_->variables_.emplace(id, variable);
  return Ok();
}

template <typename T>
Result DynamicBayesianNetwork_<T>::Add(std::shared_ptr<const Constraint_<T>> constraint) {
  const auto& id = constraint->Id();
  if (impl_->constraints_.count(id) > 0) {
    return Err(Error::kFailure, fmt::format("constraint '{}' already exists in the network", id.name));
  }
  // Validate every referenced variable before touching any adjacency list, so
  // that a failed demand leaves the graph untouched.
  if (constraint->HasConditioningVariables()) {
    for (const auto& variable_id : constraint->ConditioningVariables()) {
      BELUGA_ESTIMATION_DEMAND(impl_->variables_.count(variable_id) != 0, "conditioning variable does not exist");
    }
  }
  if (constraint->HasDependentVariables()) {
    for (const auto& variable_id : constraint->DependentVariables()) {
      BELUGA_ESTIMATION_DEMAND(impl_->variables_.count(variable_id) != 0, "dependent variable does not exist");
    }
  }
  if (constraint->HasConditioningVariables()) {
    for (const auto& variable_id : constraint->ConditioningVariables()) {
      impl_->outgoing_adjacency_list_[variable_id].push_back(id);
    }
  }
  if (constraint->HasDependentVariables()) {
    for (const auto& variable_id : constraint->DependentVariables()) {
      impl_->incoming_adjacency_list_[variable_id].push_back(id);
    }
  }
  impl_->constraint_indices_[std::string{DynamicBayesianNetwork_<T>::kDefaultName}].insert(id);
  impl_->constraint_indices_[id.name].insert(id);
  impl_->constraints_.emplace(id, constraint);
  return Ok();
}

template <typename T>
Result DynamicBayesianNetwork_<T>::Add(std::shared_ptr<const AbstractValue> value) {
  const auto& id = value->Id();
  if (impl_->values_.count(id) > 0) {
    return Err(Error::kFailure, fmt::format("value '{}' already exists in the network", id.name));
  }
  impl_->value_indices_[std::string{DynamicBayesianNetwork_<T>::kDefaultName}].insert(id);
  impl_->value_indices_[id.name].insert(id);
  impl_->values_.emplace(id, value);
  return Ok();
}

template <typename T>
Result_<std::shared_ptr<const Variable_<T>>> DynamicBayesianNetwork_<T>::MaybeAdd(
    std::shared_ptr<const Variable_<T>> variable) {
  const auto& id = variable->Id();
  auto it = impl_->variables_.find(id);
  if (it != impl_->variables_.end()) {
    // The stored variable must be exactly the kind that was proposed; a
    // disagreement means two parts of the system disagree about what it is.
    if (typeid(*it->second) != typeid(*variable)) {
      return Err(Error::kFailure, fmt::format("variable '{}' already exists with a different type", id.name));
    }
    return it->second;
  }
  impl_->incoming_adjacency_list_.emplace(id, std::vector<Identifier>{});
  impl_->outgoing_adjacency_list_.emplace(id, std::vector<Identifier>{});
  impl_->variable_indices_[std::string{DynamicBayesianNetwork_<T>::kDefaultName}].insert(id);
  impl_->variable_indices_[id.name].insert(id);
  impl_->variables_.emplace(id, variable);
  return variable;
}

template <typename T>
Result_<std::shared_ptr<const Constraint_<T>>> DynamicBayesianNetwork_<T>::MaybeAdd(
    std::shared_ptr<const Constraint_<T>> constraint) {
  const auto& id = constraint->Id();
  auto it = impl_->constraints_.find(id);
  if (it != impl_->constraints_.end()) {
    // The stored constraint must cover the same variables as the proposed one;
    // a disagreement means two parts of the system disagree about what it is.
    const bool equivalent = ranges::equal(it->second->ConditioningVariables(), constraint->ConditioningVariables()) &&
                            ranges::equal(it->second->DependentVariables(), constraint->DependentVariables());
    if (!equivalent) {
      return Err(Error::kFailure, fmt::format("constraint '{}' already exists with different variables", id.name));
    }
    return it->second;
  }
  // Validate every referenced variable before touching any adjacency list, so
  // that a failed demand leaves the graph untouched.
  if (constraint->HasConditioningVariables()) {
    for (const auto& variable_id : constraint->ConditioningVariables()) {
      BELUGA_ESTIMATION_DEMAND(impl_->variables_.count(variable_id) != 0, "conditioning variable does not exist");
    }
  }
  if (constraint->HasDependentVariables()) {
    for (const auto& variable_id : constraint->DependentVariables()) {
      BELUGA_ESTIMATION_DEMAND(impl_->variables_.count(variable_id) != 0, "dependent variable does not exist");
    }
  }
  if (constraint->HasConditioningVariables()) {
    for (const auto& variable_id : constraint->ConditioningVariables()) {
      impl_->outgoing_adjacency_list_[variable_id].push_back(id);
    }
  }
  if (constraint->HasDependentVariables()) {
    for (const auto& variable_id : constraint->DependentVariables()) {
      impl_->incoming_adjacency_list_[variable_id].push_back(id);
    }
  }
  impl_->constraint_indices_[std::string{DynamicBayesianNetwork_<T>::kDefaultName}].insert(id);
  impl_->constraint_indices_[id.name].insert(id);
  impl_->constraints_.emplace(id, constraint);
  return constraint;
}

template <typename T>
Result_<std::shared_ptr<const AbstractValue>> DynamicBayesianNetwork_<T>::MaybeAdd(
    std::shared_ptr<const AbstractValue> value) {
  const auto& id = value->Id();
  auto it = impl_->values_.find(id);
  if (it != impl_->values_.end()) {
    // The stored value must be exactly the kind that was proposed.
    if (typeid(*it->second) != typeid(*value)) {
      return Err(Error::kFailure, fmt::format("value '{}' already exists with a different type", id.name));
    }
    return it->second;
  }
  impl_->value_indices_[std::string{DynamicBayesianNetwork_<T>::kDefaultName}].insert(id);
  impl_->value_indices_[id.name].insert(id);
  impl_->values_.emplace(id, value);
  return value;
}

template <typename T>
Result DynamicBayesianNetwork_<T>::Update(std::shared_ptr<const Variable_<T>> variable) {
  const auto& id = variable->Id();
  if (impl_->variables_.count(id) == 0) {
    return Err(Error::kFailure, fmt::format("variable '{}' does not exist in the network", id.name));
  }
  impl_->variables_.insert_or_assign(id, variable);
  return Ok();
}

template <typename T>
Result DynamicBayesianNetwork_<T>::Update(std::shared_ptr<const Constraint_<T>> constraint) {
  const auto& id = constraint->Id();
  if (impl_->constraints_.count(id) == 0) {
    return Err(Error::kFailure, fmt::format("constraint '{}' does not exist in the network", id.name));
  }
  auto prev_constraint = impl_->constraints_.at(id);
  const bool equivalent_constraint =
      ranges::equal(prev_constraint->ConditioningVariables(), constraint->ConditioningVariables()) &&
      ranges::equal(prev_constraint->DependentVariables(), constraint->DependentVariables());
  BELUGA_ESTIMATION_DEMAND(equivalent_constraint, "constraints are not equivalent");
  impl_->constraints_.insert_or_assign(id, constraint);
  return Ok();
}

template <typename T>
Result DynamicBayesianNetwork_<T>::Update(std::shared_ptr<const AbstractValue> value) {
  const auto& id = value->Id();
  if (impl_->values_.count(id) == 0) {
    return Err(Error::kFailure, fmt::format("value '{}' does not exist in the network", id.name));
  }
  impl_->values_.insert_or_assign(id, value);
  return Ok();
}

template <typename T>
Result DynamicBayesianNetwork_<T>::Remove(std::shared_ptr<const Variable_<T>> variable) {
  const auto& id = variable->Id();
  if (impl_->variables_.count(id) == 0) {
    return Err(Error::kFailure, fmt::format("variable '{}' does not exist in the network", id.name));
  }
  BELUGA_ESTIMATION_DEMAND(impl_->incoming_adjacency_list_.at(id).empty(), "variable has incoming constraints");
  BELUGA_ESTIMATION_DEMAND(impl_->outgoing_adjacency_list_.at(id).empty(), "variable has outgoing constraints");
  impl_->incoming_adjacency_list_.erase(id);
  impl_->outgoing_adjacency_list_.erase(id);
  impl_->variable_indices_[std::string{DynamicBayesianNetwork_<T>::kDefaultName}].erase(id);
  impl_->variable_indices_[id.name].erase(id);
  impl_->variables_.erase(id);
  return Ok();
}

template <typename T>
Result DynamicBayesianNetwork_<T>::Remove(std::shared_ptr<const Constraint_<T>> constraint) {
  const auto& id = constraint->Id();
  if (impl_->constraints_.count(id) == 0) {
    return Err(Error::kFailure, fmt::format("constraint '{}' does not exist in the network", id.name));
  }
  if (constraint->HasConditioningVariables()) {
    for (const auto& variable_id : constraint->ConditioningVariables()) {
      // Invariant: a constraint's referenced variables exist for as long as the
      // constraint does, since Remove(Variable) refuses to delete connected
      // variables.
      BELUGA_ESTIMATION_ASSERT(impl_->outgoing_adjacency_list_.count(variable_id) != 0);
      auto& adjacency_list = impl_->outgoing_adjacency_list_.at(variable_id);
      if (auto it = std::remove(adjacency_list.begin(), adjacency_list.end(), id); it != adjacency_list.end()) {
        adjacency_list.erase(it, adjacency_list.end());
      }
    }
  }
  if (constraint->HasDependentVariables()) {
    for (const auto& variable_id : constraint->DependentVariables()) {
      BELUGA_ESTIMATION_ASSERT(impl_->incoming_adjacency_list_.count(variable_id) != 0);
      auto& adjacency_list = impl_->incoming_adjacency_list_.at(variable_id);
      if (auto it = std::remove(adjacency_list.begin(), adjacency_list.end(), id); it != adjacency_list.end()) {
        adjacency_list.erase(it, adjacency_list.end());
      }
    }
  }
  impl_->constraint_indices_[std::string{DynamicBayesianNetwork_<T>::kDefaultName}].erase(id);
  impl_->constraint_indices_[id.name].erase(id);
  impl_->constraints_.erase(id);
  return Ok();
}

template <typename T>
Result DynamicBayesianNetwork_<T>::Remove(std::shared_ptr<const AbstractValue> value) {
  const auto& id = value->Id();
  if (impl_->values_.count(id) == 0) {
    return Err(Error::kFailure, fmt::format("value '{}' does not exist in the network", id.name));
  }
  impl_->value_indices_[std::string{DynamicBayesianNetwork_<T>::kDefaultName}].erase(id);
  impl_->value_indices_[id.name].erase(id);
  impl_->values_.erase(id);
  return Ok();
}

template <typename T>
void DynamicBayesianNetwork_<T>::Clear() {
  impl_->incoming_adjacency_list_.clear();
  impl_->outgoing_adjacency_list_.clear();
  impl_->variable_indices_.clear();
  impl_->constraint_indices_.clear();
  impl_->value_indices_.clear();
  impl_->variables_.clear();
  impl_->constraints_.clear();
  impl_->values_.clear();
}

template <typename T>
bool DynamicBayesianNetwork_<T>::Accept(DynamicBayesianNetworkVisitor_<T>& visitor) const {
  std::unordered_map<Identifier, std::size_t> in_degree;
  std::set<Identifier, TimeAwareIdentifierCompare> ready_queue;

  for (const auto& [id, _] : impl_->variables_) {
    in_degree[id] = impl_->incoming_adjacency_list_.at(id).size();
    if (in_degree[id] == 0) {
      ready_queue.insert(id);
    }
  }

  for (const auto& [id, constraint] : impl_->constraints_) {
    in_degree[id] = constraint->HasConditioningVariables() ? constraint->ConditioningVariables().size() : 0;
    if (in_degree[id] == 0) {
      ready_queue.insert(id);
    }
  }

  for (const auto& [id, _] : impl_->values_) {
    ready_queue.insert(id);
  }

  std::size_t visited_count = 0;

  while (!ready_queue.empty()) {
    auto it = ready_queue.begin();
    Identifier current = *it;
    ready_queue.erase(it);
    visited_count++;

    if (auto val_it = impl_->values_.find(current); val_it != impl_->values_.end()) {
      visitor.Visit(*val_it->second);
    } else if (auto var_it = impl_->variables_.find(current); var_it != impl_->variables_.end()) {
      visitor.Visit(*var_it->second);
      for (const auto& target_constraint : impl_->outgoing_adjacency_list_.at(current)) {
        if (--in_degree[target_constraint] == 0) {
          ready_queue.insert(target_constraint);
        }
      }
    } else if (auto const_it = impl_->constraints_.find(current); const_it != impl_->constraints_.end()) {
      visitor.Visit(*const_it->second);
      if (const_it->second->HasDependentVariables()) {
        for (const auto& target_var : const_it->second->DependentVariables()) {
          if (--in_degree[target_var] == 0) {
            ready_queue.insert(target_var);
          }
        }
      }
    }
  }

  return visited_count < impl_->variables_.size() + impl_->constraints_.size() + impl_->values_.size();
}

template class DynamicBayesianNetwork_<double>;
template class DynamicBayesianNetwork_<float>;

}  // namespace beluga_estimation

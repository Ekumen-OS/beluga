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

#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation {

template <typename T>
class Variable_;

template <typename T>
class Constraint_;

class AbstractValue;

/// @brief Interface for visiting elements in a Dynamic Bayesian Network.
template <typename T>
class DynamicBayesianNetworkVisitor_ {
 public:
  virtual ~DynamicBayesianNetworkVisitor_() = default;

  virtual void Visit(const Variable_<T>& variable) = 0;
  virtual void Visit(const Constraint_<T>& constraint) = 0;
  virtual void Visit(const AbstractValue& value) = 0;
};

using DynamicBayesianNetworkVisitor = DynamicBayesianNetworkVisitor_<DefaultScalar>;

}  // namespace beluga_estimation

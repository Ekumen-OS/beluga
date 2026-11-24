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

#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>

#include <beluga_estimation/core/constraint.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/core/network_visitor.hpp>
#include <beluga_estimation/core/value.hpp>
#include <beluga_estimation/core/variable.hpp>

namespace beluga_estimation {

namespace detail {

// SFINAE check for the existence of a metadata() method.
template <typename T, typename = void>
struct HasMetadata : std::false_type {};

template <typename T>
struct HasMetadata<T, std::void_t<decltype(std::declval<const T&>().metadata())>> : std::true_type {};

// Helper to get the label: uses metadata() if available, otherwise falls back to Id().
template <typename T>
std::string GetNodeLabel(const T& node) {
  std::stringstream ss;
  if constexpr (HasMetadata<T>::value) {
    ss << node.metadata();
  } else {
    ss << node.Id();
  }
  return ss.str();
}

}  // namespace detail

/// @brief A utility that exports a Dynamic Bayesian Network to a Graphviz .dot format.
/// @tparam T The underlying data type for the network variables and constraints.
template <typename T>
class GraphvizDumper_ {
 public:
  /// @brief Dumps the Dynamic Bayesian Network to the given output stream in Graphviz format.
  /// @param os The output stream to write the Graphviz code to.
  /// @param dbn The network to dump.
  /// @return A reference to the output stream.
  std::ostream& Dump(std::ostream& os, const DynamicBayesianNetwork_<T>& dbn) const {
    os << "digraph DBN {\n";
    os << "  node [shape=ellipse];\n";
    Visitor visitor{os};
    dbn.Accept(visitor);
    os << "}\n";
    return os;
  }

  /// @brief Dumps the Dynamic Bayesian Network to a string in Graphviz format.
  /// @param dbn The network to dump.
  /// @return A string containing the Graphviz code.
  std::string Dump(const DynamicBayesianNetwork_<T>& dbn) const {
    std::stringstream ss;
    Dump(ss, dbn);
    return ss.str();
  }

 private:
  /// @brief Internal visitor used to traverse the network and generate Graphviz code.
  class Visitor : public DynamicBayesianNetworkVisitor_<T> {
   public:
    explicit Visitor(std::ostream& os) : os_(os) {}

    void Visit(const Variable_<T>& variable) override {
      os_ << "  \"" << variable.Id() << "\" [label=\"" << detail::GetNodeLabel(variable) << "\\n(Variable)\"];\n";
    }

    void Visit(const Constraint_<T>& constraint) override {
      os_ << "  \"" << constraint.Id() << "\" [label=\"" << detail::GetNodeLabel(constraint)
          << "\\n(Constraint)\", shape=box];\n";

      if (constraint.HasConditioningVariables()) {
        for (const auto& cond_id : constraint.ConditioningVariables()) {
          os_ << "  \"" << cond_id << "\" -> \"" << constraint.Id() << "\";\n";
        }
      }

      if (constraint.HasDependentVariables()) {
        for (const auto& dep_id : constraint.DependentVariables()) {
          os_ << "  \"" << constraint.Id() << "\" -> \"" << dep_id << "\";\n";
        }
      }
    }

    void Visit(const AbstractValue& value) override {
      os_ << "  \"" << value.Id() << "\" [label=\"" << detail::GetNodeLabel(value) << "\\n(Value)\", style=dashed];\n";
    }

   private:
    std::ostream& os_;
  };
};

using GraphvizDumper = GraphvizDumper_<DefaultScalar>;

}  // namespace beluga_estimation

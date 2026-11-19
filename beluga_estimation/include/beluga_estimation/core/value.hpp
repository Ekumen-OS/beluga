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

#include <utility>

#include <beluga_estimation/core/identifiable.hpp>

namespace beluga_estimation {

/// @brief Base class for all values in the Bayesian network.
///
/// Inherits from Identifiable to provide a unique identifier and name.
class AbstractValue : public Identifiable {
 public:
  using Identifiable::Identifiable;
};

/// @brief A concrete value type that stores data of a specified type.
///
/// This class template wraps a piece of data, making it identifiable within the
/// Bayesian network. It provides methods to access the underlying data.
///
/// @tparam T The type of data stored in the value.
template <typename T>
class Value : public AbstractValue {
 public:
  /// @brief Constructs a Value object with the given data.
  /// @param id The identifier for this data.
  /// @param data The data to store.
  explicit Value(Identifier id, T data) : AbstractValue(std::move(id)), data_(std::move(data)) {}

  /// @brief Returns a constant reference to the stored data.
  /// @return A constant reference to the data.
  const T& GetData() const { return data_; }

 private:
  T data_;
};

}  // namespace beluga_estimation

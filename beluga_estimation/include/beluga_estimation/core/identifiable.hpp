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

#include <string>
#include <utility>

#include <beluga_estimation/core/identifier.hpp>

namespace beluga_estimation {

/**
 * @brief Base class for objects that possess a unique identifier.
 *
 * Variables and Constraints inherit from this to be indexable and managed
 * within the Dynamic Bayesian Network (DBN).
 */
class Identifiable {
 public:
  explicit Identifiable(Identifier id) : id_(std::move(id)) {}
  virtual ~Identifiable() = default;

  const Identifier& Id() const { return id_; }

 private:
  Identifier id_;
};

}  // namespace beluga_estimation

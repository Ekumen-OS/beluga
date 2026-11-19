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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include <beluga_estimation/common/registry.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>

namespace beluga_estimation {

/**
 * @brief Creates a single instance of type `T` from middleware parameters.
 *
 * Reads the type name from the parameter `<prefix>type` and uses the registry
 * to instantiate the corresponding type. The instance is created with the
 * provided middleware and parameter prefix.
 *
 * @tparam T The base type to instantiate (e.g., BayesianOperation).
 * @param middleware The middleware interface for parameter access and dependency injection.
 * @param prefix The parameter prefix (e.g., "solver." results in reading "solver.type").
 * @return A result containing the instance, or an error if the type is not registered.
 */
template <class T>
Result_<std::unique_ptr<T>> MakeInstanceFromParams(
    std::shared_ptr<MiddlewareInterface> middleware,
    const std::string& prefix) {
  auto registry = Registry<T>::Instance();
  const std::string type_name = middleware->DeclareParameter<std::string>(prefix + "type");
  if (!registry->HasType(type_name)) {
    return Err(Error::kFailure, fmt::format("type '{}' is not registered", type_name));
  }
  return registry->MakeInstance(type_name, middleware, prefix);
}

/**
 * @brief Creates multiple instances of type `T` from middleware parameters.
 *
 * For each name in the provided list, constructs a full parameter prefix by
 * concatenating `base_prefix + name + "."`, then reads the type name from the
 * parameter `<full_prefix>type` and uses the registry to instantiate the
 * corresponding type. All instances are created with the provided middleware.
 *
 * @tparam T The base type to instantiate (e.g., BayesianOperation).
 * @param middleware The middleware interface for parameter access and dependency injection.
 * @param names A vector of stage names (e.g., ["predictor", "corrector"]).
 * @param base_prefix The base parameter prefix (e.g., "init." results in "init.predictor.", "init.corrector.").
 * @return A result containing the instances vector, or an error if any type is not registered.
 */
template <class T>
Result_<std::vector<std::shared_ptr<T>>> MakeInstancesFromParams(
    std::shared_ptr<MiddlewareInterface> middleware,
    const std::vector<std::string>& names,
    const std::string& base_prefix) {
  auto registry = Registry<T>::Instance();
  std::vector<std::shared_ptr<T>> instances;
  for (const auto& name : names) {
    const std::string full_prefix = base_prefix + name + ".";
    const std::string type_name = middleware->DeclareParameter<std::string>(full_prefix + "type");
    if (!registry->HasType(type_name)) {
      return Err(Error::kFailure, fmt::format("type '{}' is not registered", type_name));
    }
    auto instance_result = registry->MakeInstance(type_name, middleware, full_prefix);
    if (!instance_result) {
      return Err(CollectError(instance_result));
    }
    instances.emplace_back(std::shared_ptr<T>(Unwrap(instance_result)));
  }
  return instances;
}

}  // namespace beluga_estimation

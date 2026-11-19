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

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>

#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/type_casts.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/estimate.hpp>
#include <beluga_estimation/core/identifier.hpp>

namespace beluga_estimation {

/// @brief Match scope for estimate store operations.
///
/// Controls whether identifier comparison is exact (including timestamp)
/// or semantic (by name only, ignoring timestamp).
enum class MatchScope {
  /// Match by name + timestamp.
  kExact,

  /// Match by name only (ignore timestamp).
  kSemantic,
};

/// @brief In-memory estimate store keyed by @ref Identifier.
///
/// Stores estimates keyed by their identifier, supporting exact and semantic
/// (name-based) lookups.
///
/// @ref Add adds or replaces an estimate. With `kExact` scope, only the
/// exact identifier is matched. With `kSemantic` scope, any existing estimates
/// with the same name (different timestamps) are removed.
///
/// @ref Get provides lookup by identifier (via @ref Identifier) or by name
/// (via @c std::string_view, returning the most recent matching entry).
/// The typed @c Get<E> overloads perform a safe downcast to the requested
/// estimate type.
///
/// @ref Remove deletes estimates by identifier, with the same scope semantics.
///
/// The store is not thread-safe.
///
/// @tparam T Scalar type (e.g. `float` or `double`).
template <typename T>
class EstimateStore_ final {
 public:
  /// @brief Default constructor. Creates an empty store.
  EstimateStore_();

  /// @brief Default destructor.
  ~EstimateStore_();

  /// @brief Adds (or replaces) an estimate by identifier.
  ///
  /// With `kExact` scope, adds or replaces the estimate for the given
  /// identifier directly. With `kSemantic` scope, also removes any existing
  /// estimate with the same name (different timestamp).
  ///
  /// @param id The identifier for the estimate.
  /// @param estimate The estimate to add. Must not be null.
  /// @param scope Controls replacement semantics.
  /// @return `true` if an existing estimate was replaced or a same-name entry
  ///   was erased.
  /// @throws std::invalid_argument if `estimate` is null.
  bool Add(Identifier id, std::shared_ptr<const Estimate_<T>> estimate, MatchScope scope = MatchScope::kExact);

  /// @brief Retrieves an estimate by its identifier.
  /// @param id The identifier to look up.
  /// @return A shared pointer to the estimate, or nullptr if not found.
  std::shared_ptr<const Estimate_<T>> Get(const Identifier& id) const;

  /// @brief Retrieves the most recent estimate with the given name.
  ///
  /// If multiple estimates share the same name, the one with the latest
  /// timestamp is returned.
  /// @param name The name to look up.
  /// @return A shared pointer to the estimate, or nullptr if not found.
  std::shared_ptr<const Estimate_<T>> GetLatest(std::string_view name) const;

  /// @brief Typed retrieval: downcasts the result of @ref Get(const Identifier&).
  ///
  /// @tparam E The concrete estimate type to downcast to.
  /// @param id The identifier to look up.
  /// @return `Ok(nullptr)` if the estimate is not found, `Ok(estimate)` on
  ///         success, or an error with `Error::kFailure` if the stored estimate
  ///         has a different type.
  template <typename E>
  Result_<std::shared_ptr<const E>> Get(const Identifier& id) const {
    return safe_dynamic_pointer_cast<const E>(Get(id), "estimate", id.name);
  }

  /// @brief Typed retrieval for @ref GetLatest(std::string_view).
  ///
  /// @tparam E The concrete estimate type to downcast to.
  /// @param name The name to look up.
  /// @return `Ok(nullptr)` if the estimate is not found, `Ok(estimate)` on
  ///         success, or an error with `Error::kFailure` if the stored estimate
  ///         has a different type.
  template <typename E>
  Result_<std::shared_ptr<const E>> GetLatest(std::string_view name) const {
    return safe_dynamic_pointer_cast<const E>(GetLatest(name), "estimate", name);
  }

  /// @brief Returns the number of semantically distinct estimate names.
  ///
  /// Each distinct estimate name represents a different estimation source.
  /// When this value is greater than 1, @ref GetLatestForVariable cannot
  /// determine the result unambiguously and will return nullptr.
  ///
  /// @return The number of unique estimate names.
  std::size_t GetNumEstimateNames() const noexcept;

  /// @brief Returns a single-pass view over the estimate names in the store.
  /// @return A forward-only view over the unique estimate names.
  single_pass_view<std::string> GetEstimateNames() const;

  /// @brief Retrieves the most recent estimate that contains a variable
  ///        matching the given identifier.
  ///
  /// When the store contains estimates from multiple semantically distinct
  /// sources (different names), the result is inherently ambiguous and this
  /// function returns nullptr. Callers should check @ref GetNumEstimateNames
  /// before calling and skip or fail when the return value is greater than 1.
  ///
  /// When the store contains a single source, iterates estimates from newest
  /// to oldest, returning the first whose @ref Estimate_::GetVariables contains
  /// a variable with the exact same @ref Identifier (both name and timestamp).
  ///
  /// @param id The full variable identifier (name + timestamp) to search for.
  /// @return A shared pointer to the matching estimate, or nullptr if not found
  ///   or ambiguous.
  std::shared_ptr<const Estimate_<T>> GetLatestForVariable(const Identifier& id) const;

  /// @brief Typed retrieval for @ref GetLatestForVariable(const Identifier&).
  /// @tparam E The concrete estimate type to downcast to.
  /// @param id The full variable identifier to search for.
  /// @return `Ok(nullptr)` if the estimate is not found or ambiguous,
  ///         `Ok(estimate)` on success, or an error with `Error::kFailure` if
  ///         the stored estimate has a different type.
  template <typename E>
  Result_<std::shared_ptr<const E>> GetLatestForVariable(const Identifier& id) const {
    return safe_dynamic_pointer_cast<const E>(GetLatestForVariable(id), "estimate", id.name);
  }

  /// @brief Removes an estimate by identifier.
  ///
  /// With `kExact` scope, removes only the exact match.
  /// With `kSemantic` scope, removes all estimates with the same name
  /// (any timestamp).
  ///
  /// @param id The identifier to remove.
  /// @param scope Controls which estimates are removed.
  /// @return The number of estimates removed.
  std::size_t Remove(const Identifier& id, MatchScope scope = MatchScope::kExact);

  /// @brief Removes all estimates from the store.
  void Clear() noexcept;

  /// @brief Returns the number of estimates currently in the store.
  std::size_t Size() const noexcept;

 private:
  class Implementation;
  std::unique_ptr<Implementation> impl_;
};

using EstimateStore = EstimateStore_<DefaultScalar>;

}  // namespace beluga_estimation

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

#include <cstddef>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

#include <range/v3/view/map.hpp>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/estimate.hpp>
#include <beluga_estimation/core/estimate_store.hpp>
#include <beluga_estimation/core/identifier.hpp>

namespace beluga_estimation {

// Orders Identifier by (name, timestamp) with transparent string_view lookup.
//
// Used by EstimateStore where semantic (name-based) operations need all
// entries with the same name to be contiguous. The name-first ordering
// clusters all timestamps for a given name together, enabling O(log n)
// find(name).
struct NameAwareIdentifierCompare {
  using is_transparent = void;

  bool operator()(const Identifier& a, const Identifier& b) const {
    return std::tie(a.name, a.timestamp) < std::tie(b.name, b.timestamp);
  }

  bool operator()(const Identifier& a, std::string_view b) const { return a.name < b; }

  bool operator()(std::string_view a, const Identifier& b) const { return a < b.name; }
};

// -----------------------------------------------------------------------------
// PIMPL state
// -----------------------------------------------------------------------------

template <typename T>
class EstimateStore_<T>::Implementation {
 public:
  // Per-name statistics tracked by the store.
  struct EstimateStats {
    std::size_t count = 0;  ///< Number of entries with this estimate name.
  };

  // The underlying storage for estimates, keyed by their identifiers.
  //
  // The map is ordered by Identifier, which allows for efficient retrieval of
  // the latest estimate for a given name by iterating in reverse order. Estimates
  // are clustered by name, then timestamps, which makes for faster estimate based
  // I/O at the expense of slower variable based lookups.
  std::map<Identifier, std::shared_ptr<const Estimate_<T>>, NameAwareIdentifierCompare> store_;

  // Per-estimate kind stats, indexed by estimate name.
  std::unordered_map<std::string, EstimateStats> stats_;
};

// -----------------------------------------------------------------------------
// Public interface
// -----------------------------------------------------------------------------

template <typename T>
EstimateStore_<T>::EstimateStore_() : impl_(std::make_unique<Implementation>()) {}

template <typename T>
EstimateStore_<T>::~EstimateStore_() = default;

template <typename T>
bool EstimateStore_<T>::Add(Identifier id, std::shared_ptr<const Estimate_<T>> estimate, MatchScope scope) {
  BELUGA_ESTIMATION_DEMAND(estimate != nullptr, "estimate must not be null");

  auto& store = impl_->store_;

  bool removed_others = false;

  if (scope == MatchScope::kSemantic) {
    auto it = store.find(id.name);
    // Leverage name-based identifier clustering in store
    while (it != store.end() && it->first.name == id.name) {
      it = store.erase(it);
      removed_others = true;
    }
    // All entries with this name have been removed; reset the stats.
    impl_->stats_.erase(id.name);
  }

  std::string name = id.name;  // Copy before move
  auto [_, inserted_new] = store.insert_or_assign(std::move(id), std::move(estimate));

  if (inserted_new) {
    ++impl_->stats_[name].count;
  }

  return removed_others || !inserted_new;
}

template <typename T>
std::shared_ptr<const Estimate_<T>> EstimateStore_<T>::Get(const Identifier& id) const {
  const auto& store = impl_->store_;
  auto it = store.find(id);
  return (it != store.end()) ? it->second : nullptr;
}

template <typename T>
std::shared_ptr<const Estimate_<T>> EstimateStore_<T>::GetLatest(std::string_view name) const {
  const auto& store = impl_->store_;
  for (auto it = store.rbegin(); it != store.rend(); ++it) {
    if (it->first.name == name) {
      return it->second;
    }
  }
  return nullptr;
}

template <typename T>
std::shared_ptr<const Estimate_<T>> EstimateStore_<T>::GetLatestForVariable(const Identifier& variable_id) const {
  // With more than one semantically distinct estimate name, the result is always ambiguous.
  if (impl_->stats_.size() > 1) {
    return nullptr;
  }

  const auto& store = impl_->store_;
  for (auto it = store.rbegin(); it != store.rend(); ++it) {
    for (const auto& var : it->second->GetVariables()) {
      if (var->Id() == variable_id) {
        return it->second;
      }
    }
  }
  return nullptr;
}

template <typename T>
std::size_t EstimateStore_<T>::GetNumEstimateNames() const noexcept {
  return impl_->stats_.size();
}

template <typename T>
single_pass_view<std::string> EstimateStore_<T>::GetEstimateNames() const {
  return impl_->stats_ | ranges::views::keys;
}

template <typename T>
std::size_t EstimateStore_<T>::Remove(const Identifier& id, MatchScope scope) {
  auto& store = impl_->store_;

  if (scope == MatchScope::kExact) {
    auto it = store.find(id);
    if (it == store.end()) {
      return 0;
    }
    auto name = it->first.name;  // Copy before erase
    store.erase(it);
    auto& stats = impl_->stats_[name];
    if (--stats.count == 0) {
      impl_->stats_.erase(name);
    }
    return 1;
  }

  // Semantic: remove all entries with the same name (any timestamp).
  auto it = store.find(id.name);

  std::size_t removed_count = 0;
  while (it != store.end() && it->first.name == id.name) {
    it = store.erase(it);
    ++removed_count;
  }

  if (removed_count > 0) {
    impl_->stats_.erase(id.name);
  }

  return removed_count;
}

template <typename T>
void EstimateStore_<T>::Clear() noexcept {
  impl_->store_.clear();
  impl_->stats_.clear();
}

template <typename T>
std::size_t EstimateStore_<T>::Size() const noexcept {
  return impl_->store_.size();
}

// -----------------------------------------------------------------------------
// Explicit instantiations
// -----------------------------------------------------------------------------

template class EstimateStore_<double>;
template class EstimateStore_<float>;

}  // namespace beluga_estimation

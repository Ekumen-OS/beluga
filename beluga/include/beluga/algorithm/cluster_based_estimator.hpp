// Copyright 2023-2024 Ekumen, Inc.
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

#ifndef BELUGA_ALGORITHM_CLUSTER_BASED_ESTIMATOR_HPP
#define BELUGA_ALGORITHM_CLUSTER_BASED_ESTIMATOR_HPP

// standard library
#include <functional>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

// external
#include <range/v3/action/sort.hpp>
#include <range/v3/algorithm/max_element.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/cache1.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/map.hpp>
#include <range/v3/view/zip.hpp>
#include <sophus/se2.hpp>
#include <sophus/types.hpp>

// project
#include <beluga/algorithm/estimation.hpp>
#include <beluga/algorithm/spatial_hash.hpp>
#include <beluga/views.hpp>

/**
 * \file
 * \brief Implementation of a cluster-based estimation algorithm.
 */

namespace beluga {

/// Create a priority queue from a map using a specified projection.
/**
 * This function template constructs a priority queue where the elements are
 * ordered by a priority value derived from the map's values. The projection
 * function is applied to each map value to determine its priority.
 *
 * \tparam Map The type of the associative container.
 * \tparam Proj The type of the projection function.
 * \param map The map containing the data to be inserted into the priority queue.
 * \param proj The projection function used to compute the priority of each element.
 * \return A priority queue where elements are ordered by the priority computed
 *         from the map's values using the projection function.
 */
template <class Map, class Proj>
[[nodiscard]] auto make_priority_queue(const Map& map, Proj&& proj) {
  struct KeyWithPriority {
    double priority;  // priority value used to order the queue (higher value first).
    std::size_t key;  // hash of the cell in the grid cell data map.

    bool operator<(const KeyWithPriority& other) const {
      return priority < other.priority;  // sort in decreasing priority order
    }
  };

  const auto make_from_pair = [proj = std::forward<Proj>(proj)](const auto& pair) {
    return KeyWithPriority{std::invoke(proj, pair.second), pair.first};
  };

  const auto range = map |                                       //
                     ranges::views::transform(make_from_pair) |  //
                     ranges::views::common;

  return std::priority_queue<KeyWithPriority>(range.begin(), range.end());
}

/// Calculates the threshold value at a specified percentile from a range.
/**
 * Find the value that is greater than the given percentage of the numbers in a range.
 *
 * \tparam Range The type of the input range containing the values.
 * \param range The input range of values from which to calculate the percentile threshold.
 * \param percentile The percentile (between 0 and 1) to calculate the threshold for.
 * \return The value at the specified percentile in the sorted range.
 */
template <class Range>
[[nodiscard]] auto calculate_percentile_threshold(Range&& range, double percentile) {
  auto values = range | ranges::to<std::vector> | ranges::actions::sort;
  return values[static_cast<std::size_t>(static_cast<double>(values.size()) * percentile)];
}

struct ParticleClusterizerImpl {
  /// A struct that holds the data of a single cell for the clusterization algorithm.
  template <class State>
  struct Cell {
    State representative_state;             ///< state of a particle that is within the cell
    double weight{0.0};                     ///< average weight of the cell
    std::size_t num_particles{0};           ///< number of particles in the cell
    std::optional<std::size_t> cluster_id;  ///< cluster id of the cell
  };

  /// A map that holds the sparse data about the particles grouped in cells.
  template <class State>
  using Map = std::unordered_map<std::size_t, Cell<State>>;

  template <class States, class Weights, class Hashes>
  [[nodiscard]] static auto make_cluster_map(States&& states, Weights&& weights, Hashes&& hashes) {
    using State = ranges::range_value_t<States>;
    Map<State> map;

    // preallocate memory with a very rough estimation of the number of grid_cells we might end up with
    map.reserve(states.size() / 5);

    // calculate the accumulated cell weight and save a single representative state for each cell
    for (const auto& [state, weight, hash] : ranges::views::zip(states, weights, hashes)) {
      auto [it, inserted] = map.try_emplace(hash, Cell<State>{});
      auto& [_, entry] = *it;
      entry.weight += weight;
      entry.num_particles++;
      if (inserted) {
        entry.representative_state = state;
      }
    }

    return map;
  }

  template <class State>
  static void normalize_and_cap_weights(Map<State>& map, double percentile) {
    auto entries = ranges::views::values(map);

    // normalize the accumulated weight by the number of particles in each cell
    // to avoid biasing the clustering algorithm towards cells that randomly end up
    // with more particles than others.
    for (auto& entry : entries) {
      assert(entry.num_particles > 0);
      entry.weight /= static_cast<double>(entry.num_particles);
    }

    const double max_weight =
        calculate_percentile_threshold(ranges::views::transform(entries, &Cell<State>::weight), percentile);

    for (auto& entry : entries) {
      entry.weight = std::min(entry.weight, max_weight);
    }
  }

  template <class State, class NeighborsFunction>
  static void assign_clusters(Map<State>& map, NeighborsFunction&& neighbors_function) {
    auto queue = make_priority_queue(map, &Cell<State>::weight);
    const auto max_priority = queue.top().priority;

    std::size_t next_cluster_id = 0;

    while (!queue.empty()) {
      const auto hash = queue.top().key;
      queue.pop();

      // any hash that comes out of the queue is known to exist in the cell data map
      auto& grid_cell = map[hash];

      // if there's no cluster id assigned to the cell, assign a new one
      if (!grid_cell.cluster_id.has_value()) {
        grid_cell.cluster_id = next_cluster_id++;
      }

      // process the neighbors of the cell; if they don't have a cluster id already assigned
      // then assign them one and add them to the queue with an inflated priority
      // to ensure they get processed ASAP before moving on to other local peaks.
      // Notice that with this algorithm each cell will go through the priority queue at most twice.

      const auto is_valid_neighbor = [&](const auto& neighbor_hash) {
        auto it = map.find(neighbor_hash);
        return (
            (it != map.end()) &&                       // is within the map
            (!it->second.cluster_id.has_value()) &&    // AND not yet mapped to a cluster
            (it->second.weight <= grid_cell.weight));  // AND has lower weight than the current cell
      };

      for (const std::size_t neighbor_hash : neighbors_function(grid_cell.representative_state) |  //
                                                 ranges::views::cache1 |                           //
                                                 ranges::views::filter(is_valid_neighbor)) {
        auto& neighbor = map[neighbor_hash];
        neighbor.cluster_id = grid_cell.cluster_id;
        queue.push({max_priority + neighbor.weight, neighbor_hash});  // reintroduce with inflated priority
      }
    }
  }
};

/// Parameters used to construct a ParticleClusterizer instance.
struct ParticleClusterizerParam {
  double spatial_hash_resolution = 0.20;   ///< clustering algorithm spatial resolution
  double angular_hash_resolution = 0.524;  ///< clustering algorithm angular resolution

  /// Cluster weight cap threshold.
  /**
   * This parameters is the upper percentile threshold used to cap the accumulated
   * weight of the cells in the grid to flatten the top of the approximated
   * density function and make the clustering algorithm more robust to estimation noise,
   * by fusing together any adjacent peaks whose weight is above the threshold.
   * The range of this parameter is 0.0 to 1.0. The 0.9 default places the
   * threshold at approximately 0.5 standard deviation from the mean, which is
   * halfway to the inflection of the normal distribution.
   * */
  double weight_cap_percentile = 0.90;
};

class ParticleClusterizer {
 public:
  explicit ParticleClusterizer(const ParticleClusterizerParam& parameters) : parameters_{parameters} {}

  [[nodiscard]] auto neighbors(const Sophus::SE2d& pose) const {
    return adjacent_grid_cells_ |  //
           ranges::views::transform([&pose](const Sophus::SE2d& neighbor_pose) { return pose * neighbor_pose; }) |
           ranges::views::transform(spatial_hash_function_);
  }

  template <class Particles>
  [[nodiscard]] auto operator()(Particles&& particles) {
    auto states = beluga::views::states(particles);
    auto weights = beluga::views::weights(particles);
    auto hashes = states | ranges::views::transform(spatial_hash_function_) | ranges::to<std::vector>;

    auto map = ParticleClusterizerImpl::make_cluster_map(states, weights, hashes);
    ParticleClusterizerImpl::normalize_and_cap_weights(map, parameters_.weight_cap_percentile);
    ParticleClusterizerImpl::assign_clusters(map, [this](const auto& state) { return neighbors(state); });

    return hashes |  //
           ranges::views::transform([&map](std::size_t hash) { return map[hash].cluster_id.value(); }) |
           ranges::to<std::vector>;
  }

 private:
  ParticleClusterizerParam parameters_;

  beluga::spatial_hash<Sophus::SE2d> spatial_hash_function_{
      parameters_.spatial_hash_resolution,  // x
      parameters_.spatial_hash_resolution,  // y
      parameters_.angular_hash_resolution   // theta
  };

  std::array<Sophus::SE2d, 6> adjacent_grid_cells_ = {
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{+parameters_.spatial_hash_resolution, 0.0}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{-parameters_.spatial_hash_resolution, 0.0}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{0.0, +parameters_.spatial_hash_resolution}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{0.0, -parameters_.spatial_hash_resolution}},
      Sophus::SE2d{Sophus::SO2d{+parameters_.angular_hash_resolution}, Sophus::Vector2d{0.0, 0.0}},
      Sophus::SE2d{Sophus::SO2d{-parameters_.angular_hash_resolution}, Sophus::Vector2d{0.0, 0.0}},
  };
};

/// Computes a cluster based estimate from a particle set.
/**
 * Particles are grouped into clusters around local maxima, and the state mean and covariance
 * of the cluster with the highest total weight is returned.
 */
template <class Particles>
[[nodiscard]] auto cluster_based_estimate(Particles&& particles, ParticleClusterizerParam parameters = {}) {
  const auto clusters = ParticleClusterizer{parameters}(particles);

  auto per_cluster_estimates =
      estimate_clusters(beluga::views::states(particles), beluga::views::weights(particles), clusters);

  if (per_cluster_estimates.empty()) {
    // hmmm... maybe the particles are too fragmented? calculate the overall mean and covariance
    return beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
  }

  const auto get_weight = [](const auto& t) { return std::get<0>(t); };
  const auto [_, mean, covariance] = *ranges::max_element(per_cluster_estimates, std::less{}, get_weight);
  return std::make_pair(mean, covariance);
}

}  // namespace beluga

#endif

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

#ifndef BELUGA_ALGORITHM_CLUSTER_BASED_ESTIMATION_HPP
#define BELUGA_ALGORITHM_CLUSTER_BASED_ESTIMATION_HPP

// standard library
#include <functional>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

// external
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
 * ordered by a priority value derived from the map's values.
 * The elements in the queue will contain a key that belongs to the map and the
 * corresponding priority.
 *
 * \tparam Map The type of the associative container.
 * \tparam Proj The type of the projection invocable.
 * \param map The map containing the data to be inserted into the priority queue.
 * \param proj The projection function used to compute the priority of each element.
 * \return A priority queue where elements are ordered by the priority computed
 *         from the map's values using the projection function.
 */
template <class Map, class Proj>
[[nodiscard]] auto make_priority_queue(const Map& map, Proj&& proj) {
  struct KeyWithPriority {
    double priority;  // priority value used to order the queue (higher value first).
    typename Map::key_type key;

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
  auto values = range | ranges::to<std::vector>;
  const auto n = static_cast<std::ptrdiff_t>(static_cast<double>(values.size()) * percentile);
  std::nth_element(values.begin(), values.begin() + n, values.end());
  return values[static_cast<std::size_t>(n)];
}

namespace clusterizer_detail {

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

/// Create a cluster map from a range of particles and their corresponding spatial hashes.
/**
 * This method will populate all the relevant fields in the map except for the cluster ID,
 * which has to be computed with a separate call to `assign_clusters`.
 *
 * \tparam States The range type for particle states.
 * \tparam Weights The range type for particle weights.
 * \tparam Hashes The range type for particle spatial hashes.
 * \param states A range of particle states.
 * \param weights A range of particle weights.
 * \param hashes A range of particle spatial hashes.
 * \return A map where each unique hash corresponds to a cell with accumulated weight, particle count, and
 * representative state.
 */
template <class States, class Weights, class Hashes>
[[nodiscard]] static auto make_cluster_map(States&& states, Weights&& weights, Hashes&& hashes) {
  using State = ranges::range_value_t<States>;
  Map<State> map;

  // Preallocate memory with a very rough estimation of the number of cells we might end up with.
  map.reserve(states.size() / 5);

  // Calculate the accumulated cell weight and save a single representative state for each cell.
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

/// Normalize weights and cap them to a given percentile.
/**
 * Given a valid cluster map, normalize the accumulated weight by the number of particles
 * in each cell to avoid biasing the clustering algorithm towards cells that randomly end up
 * with more particles than others.
 *
 * Then cap the values to a given percentile to flatten the top of the approximated
 * density function and make the clustering algorithm more robust to estimation noise,
 * by fusing together any adjacent peaks whose weight is above the threshold.
 *
 * \tparam State The state type of the cells in the map.
 * \param map A reference to the map where cells are stored.
 * \param percentile The percentile threshold for capping the weights.
 */
template <class State>
static void normalize_and_cap_weights(Map<State>& map, double percentile) {
  auto entries = ranges::views::values(map);

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

/// Assign cluster ids to an existing cluster map.
/**
 * This function implements a clustering algorithm that assigns cluster IDs to cells in a map
 * based on their spatial relationships.
 *
 * Notice that with this algorithm each cell will go through the priority queue at most twice.
 *
 * \tparam State The state type of the cells in the map.
 * \tparam NeighborsFunction A callable object that, given a state, returns a range of neighboring cell hashes.
 * \param map A reference to the map where cells are stored.
 * \param neighbors_function A function that returns neighboring cell hashes for a given state.
 */
template <class State, class NeighborsFunction>
static void assign_clusters(Map<State>& map, NeighborsFunction&& neighbors_function) {
  auto queue = make_priority_queue(map, &Cell<State>::weight);
  const auto max_priority = queue.top().priority;

  std::size_t next_cluster_id = 0;

  // Process cells in order of descending weight.
  while (!queue.empty()) {
    const auto hash = queue.top().key;
    queue.pop();

    // The hash is guaranteed to exist in the map.
    auto& cell = map[hash];

    // If there's no cluster id assigned to the cell, assign a new one.
    if (!cell.cluster_id.has_value()) {
      cell.cluster_id = next_cluster_id++;
    }

    // Process the neighbors of the cell; if they don't have a cluster id already assigned
    // then assign them one and add them to the queue with an inflated priority
    // to ensure they get processed ASAP before moving on to other local peaks.
    // Notice that with this algorithm each cell will go through the priority queue at most twice.

    // Define a lambda to check if a neighboring cell is valid for clustering.
    const auto is_valid_neighbor = [&](const auto& neighbor_hash) {
      auto it = map.find(neighbor_hash);
      return (
          (it != map.end()) &&                     // is within the map
          (!it->second.cluster_id.has_value()) &&  // AND not yet mapped to a cluster
          (it->second.weight <= cell.weight));     // AND has lower weight than the current cell
    };

    // Process the neighbors of the current cell.
    for (const std::size_t neighbor_hash : neighbors_function(cell.representative_state) |  //
                                               ranges::views::cache1 |                      //
                                               ranges::views::filter(is_valid_neighbor)) {
      auto& neighbor = map[neighbor_hash];
      neighbor.cluster_id = cell.cluster_id;
      queue.push({max_priority + neighbor.weight, neighbor_hash});  // reintroduce with inflated priority
    }
  }
}

}  // namespace clusterizer_detail

/// Parameters used to construct a ParticleClusterizer instance.
struct ParticleClusterizerParam {
  double linear_hash_resolution = 0.20;    ///< clustering algorithm linear resolution
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

/// Particle clusterizer implementation.
class ParticleClusterizer {
 public:
  /// Constructor that initializes the ParticleClusterizer with given parameters.
  explicit ParticleClusterizer(const ParticleClusterizerParam& parameters) : parameters_{parameters} {}

  /// Computes the neighboring cells for a given pose using the spatial hash function.
  /**
   * \param pose The pose for which neighboring cells are computed.
   * \return A range of spatial hashes corresponding to the neighboring cells.
   */
  [[nodiscard]] auto neighbors(const Sophus::SE2d& pose) const {
    return adjacent_grid_cells_ |  //
           ranges::views::transform([&pose](const Sophus::SE2d& neighbor_pose) { return pose * neighbor_pose; }) |
           ranges::views::transform(spatial_hash_function_);
  }

  /// Clusters the given particles based on their states and weights.
  /**
   * \tparam Particles The type of the particles range.
   * \param particles A range of particles to be clustered.
   * \return A vector of cluster IDs corresponding to the input particles.
   */
  template <class Particles>
  [[nodiscard]] auto operator()(Particles&& particles) {
    auto states = beluga::views::states(particles);
    auto weights = beluga::views::weights(particles);
    auto hashes = states | ranges::views::transform(spatial_hash_function_) | ranges::to<std::vector>;

    auto map = clusterizer_detail::make_cluster_map(states, weights, hashes);
    clusterizer_detail::normalize_and_cap_weights(map, parameters_.weight_cap_percentile);
    clusterizer_detail::assign_clusters(map, [this](const auto& state) { return neighbors(state); });

    return hashes |  //
           ranges::views::transform([&map](std::size_t hash) { return map[hash].cluster_id.value(); }) |
           ranges::to<std::vector>;
  }

 private:
  ParticleClusterizerParam parameters_;  ///< Parameters for the particle clusterizer.

  beluga::spatial_hash<Sophus::SE2d> spatial_hash_function_{
      parameters_.linear_hash_resolution,  // x
      parameters_.linear_hash_resolution,  // y
      parameters_.angular_hash_resolution  // theta
  };

  std::array<Sophus::SE2d, 6> adjacent_grid_cells_ = {
      ///< Adjacent grid cells for neighbor calculation.
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{+parameters_.linear_hash_resolution, 0.0}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{-parameters_.linear_hash_resolution, 0.0}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{0.0, +parameters_.linear_hash_resolution}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{0.0, -parameters_.linear_hash_resolution}},
      Sophus::SE2d{Sophus::SO2d{+parameters_.angular_hash_resolution}, Sophus::Vector2d{0.0, 0.0}},
      Sophus::SE2d{Sophus::SO2d{-parameters_.angular_hash_resolution}, Sophus::Vector2d{0.0, 0.0}},
  };
};

/// For each cluster, estimate the mean and covariance of the states that belong to it.
/**
 * \tparam States Range type of the states.
 * \tparam Weights Range type of the weights.
 * \tparam Hashes Range type of the hashes.
 * \param states Range containing the states of the particles.
 * \param weights Range containing the weights of the particles.
 * \param clusters Cluster ids of the particles.
 * \return A vector of elements, containing the weight, mean and covariance of each cluster, in no particular order.
 */
template <class States, class Weights, class Clusters>
[[nodiscard]] auto estimate_clusters(States&& states, Weights&& weights, Clusters&& clusters) {
  using State = typename ranges::range_value_t<States>;
  using Weight = typename ranges::range_value_t<Weights>;
  using Cluster = typename ranges::range_value_t<Clusters>;

  using EstimateState = std::decay_t<decltype(std::get<0>(beluga::estimate(states, weights)))>;
  using EstimateCovariance = std::decay_t<decltype(std::get<1>(beluga::estimate(states, weights)))>;

  static_assert(std::is_same_v<State, EstimateState>);

  struct Particle {
    State state;
    Weight weight;
    Cluster cluster;

    /// Convenient factory method to pass to `zip_with`.
    static constexpr auto construct(const State& s, Weight w, Cluster c) { return Particle{s, w, c}; }
  };

  struct Estimate {
    Weight weight;
    EstimateState mean;
    EstimateCovariance covariance;

    /// Constructor used with `emplace_back`.
    constexpr Estimate(Weight w, EstimateState m, EstimateCovariance c)
        : weight{w}, mean{std::move(m)}, covariance{std::move(c)} {}
  };

  // For each cluster found, estimate the mean and covariance of the states that belong to it.
  const auto unique_clusters = clusters | ranges::to<std::unordered_set>;
  auto estimates = std::vector<Estimate>{};
  estimates.reserve(unique_clusters.size());

  for (const auto cluster : unique_clusters) {
    auto particles = ranges::views::zip_with(&Particle::construct, states, weights, clusters) |          //
                     ranges::views::cache1 |                                                             //
                     ranges::views::filter([cluster](const auto& p) { return p.cluster == cluster; }) |  //
                     ranges::to<std::vector>;

    if (particles.size() < 2) {
      // If there's only one sample in the cluster we can't estimate the covariance.
      continue;
    }

    auto filtered_states = particles | ranges::views::transform(&Particle::state);
    auto filtered_weights = particles | ranges::views::transform(&Particle::weight);
    const auto [mean, covariance] = estimate(filtered_states, filtered_weights);
    const auto total_weight = ranges::accumulate(filtered_weights, 0.0);
    estimates.emplace_back(total_weight, std::move(mean), std::move(covariance));
  }

  return estimates;
}

/// Computes a cluster-based estimate from a particle set.
/**
 * Particles are grouped into clusters around local maxima. The state mean and covariance
 * of the cluster with the highest total weight is returned. If no clusters are found,
 * the overall mean and covariance of the particles are calculated and returned.
 *
 * \tparam Particles The type of the particles range.
 * \param particles A range of particles to be clustered and estimated.
 * \param parameters Parameters for the particle clusterizer (optional).
 * \return A pair consisting of the state mean and covariance of the cluster with the highest total weight.
 */
template <class Particles>
[[nodiscard]] auto cluster_based_estimate(Particles&& particles, ParticleClusterizerParam parameters = {}) {
  const auto clusters = ParticleClusterizer{parameters}(particles);

  auto per_cluster_estimates =
      estimate_clusters(beluga::views::states(particles), beluga::views::weights(particles), clusters);

  if (per_cluster_estimates.empty()) {
    // hmmm... maybe the particles are too fragmented? calculate the overall mean and covariance.
    return beluga::estimate(
        beluga::views::states(particles),  // namespace beluga
        beluga::views::weights(particles));
  }

  const auto [_, mean, covariance] =
      *ranges::max_element(per_cluster_estimates, std::less{}, [](const auto& e) { return e.weight; });

  return std::make_pair(mean, covariance);
}

}  // namespace beluga

#endif

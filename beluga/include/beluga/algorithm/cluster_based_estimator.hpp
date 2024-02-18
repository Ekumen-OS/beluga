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

#ifndef BELUGA_ESTIMATION_CLUSTER_BASED_ESTIMATOR_HPP
#define BELUGA_ESTIMATION_CLUSTER_BASED_ESTIMATOR_HPP

// standard library
#include <algorithm>
#include <initializer_list>
#include <numeric>
#include <optional>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// external
#include <range/v3/action/sort.hpp>
#include <range/v3/algorithm/max_element.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/cache1.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/zip.hpp>
#include <sophus/se2.hpp>
#include <sophus/types.hpp>

// project
#include <beluga/algorithm/estimation.hpp>
#include <beluga/algorithm/spatial_hash.hpp>
#include <beluga/views.hpp>

/**
 * \file
 * \brief Implementation of a cluster-base estimator mixin.
 */

namespace beluga {

/// \brief A struct that holds the data of a single cell in the grid.
struct GridCellData {
  double weight{0.0};                         ///< average weight of the cell
  std::size_t num_particles{0};               ///< number of particles in the cell
  Sophus::SE2d representative_pose_in_world;  ///< state of a particle that is within the cell
  std::optional<std::size_t> cluster_id;      ///< cluster id of the cell
};

/// \brief A map that holds the sparse data about the particles grouped in cells. Used by the clusterization algorithm.
using GridCellDataMap2D = std::unordered_map<std::size_t, GridCellData>;

/// \brief Populate the grid cell data map with the data from the particles and their weights.
/// \tparam GridCellDataMapType Type of the grid cell data map.
/// \tparam Range Type of the states range.
/// \tparam Weights Type of the weights range.
/// \tparam Hashes Type of the hashes range.
/// \param states Range containing the states of the particles.
/// \param weights Range containing the weights of the particles.
/// \param hashes Range containing the hashes of the particles.
/// \return New instance of the grid cell data map populated with the information from the states.
template <class GridCellDataMapType, class Range, class Weights, class Hashes>
[[nodiscard]] auto populate_grid_cell_data_from_particles(Range&& states, Weights&& weights, const Hashes& hashes) {
  GridCellDataMapType grid_cell_data;

  // preallocate memory with a very rough estimation of the number of grid_cells we might end up with
  grid_cell_data.reserve(states.size() / 5);

  // calculate the accumulated cell weight and save a single representative_pose_in_world for each cell
  for (const auto& [state, weight, hash] : ranges::views::zip(states, weights, hashes)) {
    auto [it, inserted] = grid_cell_data.try_emplace(hash, GridCellData{});
    it->second.weight += weight;
    it->second.num_particles++;
    if (inserted) {
      it->second.representative_pose_in_world = state;
    }
  }

  // normalize the accumulated weight by the number of particles in each cell
  // to avoid biasing the clustering algorithm towards cells that randomly end up
  // with more particles than others.
  for (auto& [hash, entry] : grid_cell_data) {
    entry.weight /= static_cast<double>(entry.num_particles);  // num_particles is guaranteed to be > 0
  }

  return grid_cell_data;
}

/// \brief Calculate the weight threshold that corresponds to a given percentile of the weights.
/// \tparam GridCellDataMapType Type of the grid cell data map.
/// \param grid_cell_data The grid cell data map.
/// \param threshold The percentile of the weights to calculate the threshold for (range: 0.0 to 1.0)
/// \return Threshold value that corresponds to the given percentile of the weights.
template <class GridCellDataMapType>
[[nodiscard]] auto calculate_percentile_weight_threshold(GridCellDataMapType&& grid_cell_data, double threshold) {
  const auto extract_weight_f = [](const auto& grid_cell) { return grid_cell.second.weight; };
  auto weights = grid_cell_data | ranges::views::transform(extract_weight_f) | ranges::to<std::vector<double>>() |
                 ranges::actions::sort;
  return weights[static_cast<std::size_t>(static_cast<double>(weights.size()) * threshold)];
}

/// \brief Cap the weight of each cell in the grid cell data map to a given value.
/// \tparam GridCellDataMapType Type of the grid cell data map.
/// \param grid_cell_data The grid cell data map.
/// \param weight_cap The maximum weight value to be assigned to each cell.
template <class GridCellDataMapType>
void cap_grid_cell_data_weights(GridCellDataMapType&& grid_cell_data, double weight_cap) {
  for (auto& [hash, entry] : grid_cell_data) {
    const auto capped_weight = std::min(entry.weight, weight_cap);
    entry.weight = capped_weight;
  }
}

/// \brief Creates the priority queue used by the clustering information from the grid cell data map.
/// \tparam GridCellDataMapType Type of the grid cell data map.
/// \param grid_cell_data The grid cell data map.
/// \return A priority queue containing the information from the grid cell data map.
template <class GridCellDataMapType>
[[nodiscard]] auto populate_priority_queue(GridCellDataMapType&& grid_cell_data) {
  struct PriorityQueueItem {
    double priority;   // priority value used to order the queue (higher value first).
    std::size_t hash;  // hash of the cell in the grid cell data map.
  };

  struct PriorityQueueItemCompare {
    constexpr bool operator()(const PriorityQueueItem& lhs, const PriorityQueueItem& rhs) const {
      return lhs.priority < rhs.priority;  // sort in decreasing priority order
    }
  };

  const auto cell_data_to_queue_item = [](const auto& grid_cell) {
    return PriorityQueueItem{grid_cell.second.weight, grid_cell.first};
  };

  auto queue_container = grid_cell_data |                                     //
                         ranges::views::transform(cell_data_to_queue_item) |  //
                         ranges::to<std::vector<PriorityQueueItem>>();        //
  return std::priority_queue<PriorityQueueItem, std::vector<PriorityQueueItem>, PriorityQueueItemCompare>(
      PriorityQueueItemCompare{}, std::move(queue_container));
}

/// \brief Function that runs the clustering algorithm and assigns a cluster id to each cell in the grid cell data map.
/// \tparam GridCellDataMapType Type of the grid cell data map.
/// \tparam Hasher Type of the hash function used to convert states into hashes.
/// \tparam Neighbors Type of the range containing the neighbors of a cell.
/// \param grid_cell_data The grid cell data map.
/// \param spatial_hash_function_ The hash object instance.
/// \param neighbors Range containing the neighbors of a cell.
/// \param weight_cap The maximum weight value to be assigned to each cell.
template <class GridCellDataMapType, class Hasher, class Neighbors>
void map_cells_to_clusters(
    GridCellDataMapType&& grid_cell_data,
    Hasher&& spatial_hash_function_,
    Neighbors&& neighbors,
    double weight_cap) {
  auto grid_cell_queue = populate_priority_queue(grid_cell_data);

  std::size_t next_cluster_id = 0;

  while (!grid_cell_queue.empty()) {
    const auto grid_cell_hash = grid_cell_queue.top().hash;
    grid_cell_queue.pop();

    // any hash that comes out of the queue is known to exist in the cell data map
    auto& grid_cell = grid_cell_data[grid_cell_hash];
    const auto& grid_cell_weight = grid_cell.weight;
    const auto& representative_pose_in_world = grid_cell.representative_pose_in_world;

    // if there's no cluster id assigned to the cell, assign it a new one
    if (!grid_cell.cluster_id.has_value()) {
      grid_cell.cluster_id = next_cluster_id++;
    }

    // process the neighbors of the cell; if they don't have a cluster id already assigned
    // then assign them one and add them to the queue with an inflated priority
    // to ensure they get processed ASAP before moving on to other local peaks.
    // Notice that with this algorithm each cell will go through the priority queue at most twice.

    const auto get_neighbor_hash = [&](const auto& neighbor_pose_in_representative) {
      return spatial_hash_function_(representative_pose_in_world * neighbor_pose_in_representative);
    };

    const auto filter_invalid_neighbors = [&](const auto& neighbor_hash) {
      auto it = grid_cell_data.find(neighbor_hash);
      return (
          (it != grid_cell_data.end()) &&            // is within the map
          (!it->second.cluster_id.has_value()) &&    // AND not yet mapped to a cluster
          (it->second.weight <= grid_cell_weight));  // AND has lower weight than the current cell
    };

    auto valid_neighbor_hashes_view =                     //
        neighbors |                                       //
        ranges::views::transform(get_neighbor_hash) |     //
        ranges::views::cache1 |                           //
        ranges::views::filter(filter_invalid_neighbors);  //

    for (const auto& neighbor_hash : valid_neighbor_hashes_view) {
      auto& neighbor = grid_cell_data[neighbor_hash];
      neighbor.cluster_id = grid_cell.cluster_id;
      const auto inflated_priority =
          weight_cap + neighbor.weight;  // since weights are capped at weight_cap, this gives us a value that is
                                         // guaranteed to be higher than any other weight from a local maximum.
      grid_cell_queue.push({inflated_priority, neighbor_hash});  // reintroduce with inflated priority
    }
  }
}

/// Parameters used to construct a ClusterBasedEstimator instance.
struct ClusterBasedStateEstimatorParam {
  double spatial_hash_resolution = 0.20;   ///< clustering algorithm spatial resolution
  double angular_hash_hesolution = 0.524;  ///< clustering algorithm angular resolution

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

/// Primary template for a cluster-based estimation algorithm.
/**
 * Particles are grouped into clusters around local maxima, and the state mean and covariance
 * of the cluster with the highest total weight is returned.
 *
 * This class implements the EstimationInterface interface
 * and satisfies \ref StateEstimatorPage.
 */
class ClusterBasedStateEstimator {
 public:
  using param_type = ClusterBasedStateEstimatorParam;  ///< Type of the parameters used to construct the estimator.

  /// Constructs a ClusterBasedStateEstimator instance.
  /**
   * \param parameters Algorithm parameters.
   */
  explicit ClusterBasedStateEstimator(const param_type& parameters) : parameters_{parameters} {}

  /// Estimate the weighted mean and covariance of largest (largest aggregated weight) cluster within the particle set.
  /**
   * \tparam Particles The type of the states range.
   * \param particles The particle set.
   * \return The weighted mean state and covariance of the largest cluster.
   */
  template <class Particles>
  [[nodiscard]] auto estimate(Particles&& particles) const;

 private:
  param_type parameters_;

  /// \brief spatial hash function used to group particles in cells
  const beluga::spatial_hash<Sophus::SE2d> spatial_hash_function_{
      parameters_.spatial_hash_resolution,  // x
      parameters_.spatial_hash_resolution,  // y
      parameters_.angular_hash_hesolution   // theta
  };

  /// \brief Adjacent grid cells used by the clustering algorithm.
  const std::vector<Sophus::SE2d> adjacent_grid_cells_ = {
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{+parameters_.spatial_hash_resolution, 0.0}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{-parameters_.spatial_hash_resolution, 0.0}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{0.0, +parameters_.spatial_hash_resolution}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{0.0, -parameters_.spatial_hash_resolution}},
      Sophus::SE2d{Sophus::SO2d{+parameters_.angular_hash_hesolution}, Sophus::Vector2d{0.0, 0.0}},
      Sophus::SE2d{Sophus::SO2d{-parameters_.angular_hash_hesolution}, Sophus::Vector2d{0.0, 0.0}},
  };
};

template <class Particles>
auto ClusterBasedStateEstimator::estimate(Particles&& particles) const {
  auto hashes = beluga::views::states(particles) | ranges::views::transform(spatial_hash_function_) |
                ranges::to<std::vector<std::size_t>>();

  auto grid_cell_data = populate_grid_cell_data_from_particles<GridCellDataMap2D>(
      beluga::views::states(particles), beluga::views::weights(particles), hashes);

  const auto weight_cap = calculate_percentile_weight_threshold(grid_cell_data, parameters_.weight_cap_percentile);

  cap_grid_cell_data_weights(grid_cell_data, weight_cap);
  map_cells_to_clusters(grid_cell_data, spatial_hash_function_, adjacent_grid_cells_, weight_cap);

  const auto cluster_from_hash = [&grid_cell_data](const std::size_t hash) {
    const auto& grid_cell = grid_cell_data[hash];
    return grid_cell.cluster_id;
  };

  const auto clusters = hashes | ranges::views::transform(cluster_from_hash) | ranges::views::common;

  auto per_cluster_estimates =
      estimate_clusters(beluga::views::states(particles), beluga::views::weights(particles), clusters);

  if (per_cluster_estimates.empty()) {
    // hmmm... maybe the particles are too fragmented? calculate the overall mean and covariance
    return beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
  }

  const auto [_, mean, covariance] =
      *ranges::max_element(per_cluster_estimates, std::less{}, [](const auto& t) { return std::get<0>(t); });
  return std::make_pair(mean, covariance);
}

}  // namespace beluga

#endif

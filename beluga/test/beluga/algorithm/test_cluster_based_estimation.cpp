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

#include <gmock/gmock.h>

#include <beluga/algorithm/cluster_based_estimation.hpp>
#include <beluga/algorithm/spatial_hash.hpp>
#include <range/v3/action/sort.hpp>
#include <range/v3/action/unique.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/unique.hpp>
#include <sophus/se2.hpp>

#include <beluga/testing/sophus_matchers.hpp>

namespace beluga {
namespace {

using testing::SE2Near;
using testing::Vector3Near;

using Eigen::Vector2d;
using Sophus::Matrix3d;
using Sophus::SE2d;
using Sophus::SO2d;

template <class Range, class Hasher>
[[nodiscard]] auto precalculate_particle_hashes(Range&& states, const Hasher& spatial_hash_function_) {
  return states | ranges::views::transform(spatial_hash_function_) | ranges::to<std::vector<std::size_t>>();
}

struct ClusterBasedEstimationDetailTesting : public ::testing::Test {
  double kSpatialHashResolution = 1.0;
  double kAngularHashResolution = Sophus::Constants<double>::pi() / 2.0;  // 90 degrees
  double kTolerance = 1e-6;

  // spatial hash function used to group particles in cells
  beluga::spatial_hash<Sophus::SE2d> spatial_hash_function{
      kSpatialHashResolution,  // x
      kSpatialHashResolution,  // y
      kAngularHashResolution   // theta
  };

  [[nodiscard]] auto generate_test_grid_cell_data_map() const {
    constexpr auto kUpperLimit = 30.0;

    typename clusterizer_detail::Map<SE2d> map;
    for (double x = 0.0; x < kUpperLimit; x += 1.0) {
      const auto weight = x;
      const auto state = SE2d{SO2d{0.}, Vector2d{x, x}};
      map.emplace(spatial_hash_function(state), clusterizer_detail::Cell<SE2d>{state, weight, 0, std::nullopt});
    }
    return map;
  }

  [[nodiscard]] static auto
  make_particle_multicluster_dataset(double xmin, double xmax, double ymin, double ymax, double step) {
    std::vector<std::pair<SE2d, beluga::Weight>> particles;

    const auto xwidth = xmax - xmin;
    const auto ywidth = ymax - ymin;

    // simulate particles in a grid with 4 (2x2) clusters with
    // different peak heights. The highest peak is the one located on the
    // upper-right.

    for (double x = step / 2.0; x <= xwidth; x += step) {
      for (double y = step / 2.0; y <= ywidth; y += step) {
        // adjust the height of each of the four peaks
        const auto k = (2 * x < xwidth ? 0.0 : 1.0) + (2 * y < ywidth ? 0.0 : 2.0) + 1.0;
        auto weight = std::abs(std::sin(2.0 * M_PI * x / xwidth)) *  //
                      std::abs(std::sin(2.0 * M_PI * y / ywidth)) *  //
                      k;
        // add a field of zeros around the peaks to ease predicting the mean and
        // covariance values of the highest peaks
        weight = std::max(0.0, weight - k / 2.0);
        particles.emplace_back(SE2d{SO2d{0.}, Vector2d{x + xmin, y + ymin}}, weight);
      }
    }

    return particles;
  }
};

TEST_F(ClusterBasedEstimationDetailTesting, ParticleHashesCalculationStep) {
  const auto s00 = SE2d{SO2d{0.0}, Vector2d{0.25, 0.25}};
  const auto s01 = SE2d{SO2d{0.0}, Vector2d{3.75, 0.75}};
  const auto s10 = SE2d{SO2d{2.0}, Vector2d{0.00, 0.00}};
  const auto s20 = SE2d{SO2d{2.0}, Vector2d{2.00, 0.00}};

  const auto states = std::vector{s00, s01, s10, s20};

  const auto hashes = precalculate_particle_hashes(states, spatial_hash_function);

  const auto hash00 = spatial_hash_function(s00);
  const auto hash01 = spatial_hash_function(s01);
  const auto hash10 = spatial_hash_function(s10);
  const auto hash20 = spatial_hash_function(s20);

  ASSERT_EQ(hashes.size(), 4);
  EXPECT_EQ(hashes[0], hash00);
  EXPECT_EQ(hashes[1], hash01);
  EXPECT_EQ(hashes[2], hash10);
  EXPECT_EQ(hashes[3], hash20);
}

TEST_F(ClusterBasedEstimationDetailTesting, GridCellDataMapGenerationStep) {
  const auto s00 = SE2d{SO2d{0.0}, Vector2d{0.25, 0.25}};  // bin 1
  const auto s01 = SE2d{SO2d{0.0}, Vector2d{0.75, 0.75}};  // bin 1
  const auto s10 = SE2d{SO2d{2.0}, Vector2d{0.00, 0.00}};  // bin 2
  const auto s20 = SE2d{SO2d{2.0}, Vector2d{2.00, 0.00}};  // bin 3

  const auto particles = std::vector<std::pair<SE2d, beluga::Weight>>{
      std::make_pair(s00, 1.5),
      std::make_pair(s01, 0.5),
      std::make_pair(s10, 1.0),
      std::make_pair(s20, 1.0),
  };

  auto states = beluga::views::states(particles);
  auto weights = beluga::views::weights(particles);
  auto hashes = states | ranges::views::transform(spatial_hash_function) | ranges::to<std::vector>;

  auto test_data = clusterizer_detail::make_cluster_map(states, weights, hashes);

  const auto hash00 = spatial_hash_function(s00);
  const auto hash10 = spatial_hash_function(s10);
  const auto hash20 = spatial_hash_function(s20);

  ASSERT_EQ(test_data.size(), 3);
  ASSERT_NE(test_data.find(hash00), test_data.end());
  ASSERT_NE(test_data.find(hash10), test_data.end());
  ASSERT_NE(test_data.find(hash20), test_data.end());

  EXPECT_EQ(test_data[hash00].weight, 2.0);
  EXPECT_EQ(test_data[hash10].weight, 1.0);
  EXPECT_EQ(test_data[hash20].weight, 1.0);

  ASSERT_THAT(test_data[hash00].representative_state, SE2Near(s00.so2(), s00.translation(), kTolerance));
  ASSERT_THAT(test_data[hash10].representative_state, SE2Near(s10.so2(), s10.translation(), kTolerance));
  ASSERT_THAT(test_data[hash20].representative_state, SE2Near(s20.so2(), s20.translation(), kTolerance));

  ASSERT_FALSE(test_data[hash00].cluster_id.has_value());
  ASSERT_FALSE(test_data[hash10].cluster_id.has_value());
  ASSERT_FALSE(test_data[hash20].cluster_id.has_value());
}

TEST_F(ClusterBasedEstimationDetailTesting, MakePriorityQueue) {
  // data preparation
  auto data = generate_test_grid_cell_data_map();

  // test proper
  auto prio_queue = make_priority_queue(data, &clusterizer_detail::Cell<SE2d>::weight);
  EXPECT_EQ(prio_queue.size(), data.size());

  // from there on the weights should be strictly decreasing
  auto prev_weight = prio_queue.top().priority;
  while (!prio_queue.empty()) {
    const auto top = prio_queue.top();
    EXPECT_GE(prev_weight, top.priority);
    prev_weight = top.priority;
    prio_queue.pop();
  }
}

TEST_F(ClusterBasedEstimationDetailTesting, MapGridCellsToClustersStep) {
  const double k_field_side = 36.0;
  const double k_half_field_side = 18.0;

  // create a map with four independent peaks
  std::vector<std::tuple<double, double, double>> coordinates;
  for (double x = 0.0; x < k_field_side; x += 1.0) {
    for (double y = 0.0; y < k_field_side; y += 1.0) {
      const auto weight = std::abs(std::sin(10.0 * x * M_PI / 180.0)) * std::abs(std::sin(10.0 * y * M_PI / 180.0));
      coordinates.emplace_back(x, y, weight);
    }
  }

  typename clusterizer_detail::Map<SE2d> map;

  for (const auto& [x, y, w] : coordinates) {
    const auto state = SE2d{SO2d{0.}, Vector2d{x, y}};
    map.emplace(spatial_hash_function(state), clusterizer_detail::Cell<SE2d>{state, w, 0, std::nullopt});
  }

  const auto neighbors = [&](const auto& state) {
    static const auto kAdjacentGridCells = {
        SE2d{SO2d{0.0}, Vector2d{+kSpatialHashResolution, 0.0}},
        SE2d{SO2d{0.0}, Vector2d{-kSpatialHashResolution, 0.0}},
        SE2d{SO2d{0.0}, Vector2d{0.0, +kSpatialHashResolution}},
        SE2d{SO2d{0.0}, Vector2d{0.0, -kSpatialHashResolution}},
    };

    return kAdjacentGridCells |  //
           ranges::views::transform([&state](const Sophus::SE2d& neighbor_pose) { return state * neighbor_pose; }) |
           ranges::views::transform(spatial_hash_function);
  };

  // test proper

  // only cells beyond the 10% weight percentile to avoid the messy border
  // between clusters beneath that threshold
  const auto ten_percent_threshold = calculate_percentile_threshold(
      map | ranges::views::values | ranges::views::transform(&clusterizer_detail::Cell<SE2d>::weight), 0.15);

  clusterizer_detail::assign_clusters(map, neighbors);

  auto cells_above_minimum_threshold_view =
      coordinates | ranges::views::filter([&](const auto& c) { return std::get<2>(c) >= ten_percent_threshold; });

  const auto right_side_cell = [&](const auto& c) { return std::get<0>(c) >= k_half_field_side; };
  const auto left_side_cell = [&](const auto& c) { return !right_side_cell(c); };
  const auto top_side_cell = [&](const auto& c) { return std::get<1>(c) >= k_half_field_side; };
  const auto bottom_side_cell = [&](const auto& c) { return !top_side_cell(c); };

  auto quadrant_1_view = cells_above_minimum_threshold_view |     //
                         ranges::views::filter(left_side_cell) |  //
                         ranges::views::filter(bottom_side_cell);
  auto quadrant_2_view = cells_above_minimum_threshold_view |      //
                         ranges::views::filter(right_side_cell) |  //
                         ranges::views::filter(bottom_side_cell);
  auto quadrant_3_view = cells_above_minimum_threshold_view |     //
                         ranges::views::filter(left_side_cell) |  //
                         ranges::views::filter(top_side_cell);
  auto quadrant_4_view = cells_above_minimum_threshold_view |      //
                         ranges::views::filter(right_side_cell) |  //
                         ranges::views::filter(top_side_cell);

  const auto coord_to_hash = [&](const auto& coords) {
    const auto& [x, y, w] = coords;
    const auto state = SE2d{SO2d{0.}, Vector2d{x, y}};
    return spatial_hash_function(state);
  };

  const auto hash_to_id = [&](const auto& hash) { return map[hash].cluster_id.value(); };

  auto quadrant_1_unique_ids = quadrant_1_view |                          //
                               ranges::views::transform(coord_to_hash) |  //
                               ranges::views::transform(hash_to_id) |     //
                               ranges::to<std::vector<std::size_t>>() |   //
                               ranges::actions::sort |                    //
                               ranges::actions::unique;                   //
  auto quadrant_2_unique_ids = quadrant_2_view |                          //
                               ranges::views::transform(coord_to_hash) |  //
                               ranges::views::transform(hash_to_id) |     //
                               ranges::to<std::vector<std::size_t>>() |   //
                               ranges::actions::sort |                    //
                               ranges::actions::unique;                   //
  auto quadrant_3_unique_ids = quadrant_3_view |                          //
                               ranges::views::transform(coord_to_hash) |  //
                               ranges::views::transform(hash_to_id) |     //
                               ranges::to<std::vector<std::size_t>>() |   //
                               ranges::actions::sort |                    //
                               ranges::actions::unique;                   //
  auto quadrant_4_unique_ids = quadrant_4_view |                          //
                               ranges::views::transform(coord_to_hash) |  //
                               ranges::views::transform(hash_to_id) |     //
                               ranges::to<std::vector<std::size_t>>() |   //
                               ranges::actions::sort |                    //
                               ranges::actions::unique;                   //

  auto full_field_unique_ids = cells_above_minimum_threshold_view |       //
                               ranges::views::transform(coord_to_hash) |  //
                               ranges::views::transform(hash_to_id) |     //
                               ranges::to<std::vector<std::size_t>>() |   //
                               ranges::actions::sort |                    //
                               ranges::actions::unique;                   //

  // check that each quadrant receives its own cluster id, and that
  // there are four clusters in total
  EXPECT_EQ(quadrant_1_unique_ids.size(), 1);
  EXPECT_EQ(quadrant_2_unique_ids.size(), 1);
  EXPECT_EQ(quadrant_3_unique_ids.size(), 1);
  EXPECT_EQ(quadrant_4_unique_ids.size(), 1);

  EXPECT_EQ(full_field_unique_ids.size(), 4);
}

TEST_F(ClusterBasedEstimationDetailTesting, ClusterStateEstimationStep) {
  const double k_field_side = 36.0;

  // create a map with four independent peaks
  auto particles = make_particle_multicluster_dataset(0.0, k_field_side, 0.0, k_field_side, 1.0);

  const auto clusters =
      ParticleClusterizer{ParticleClusterizerParam{kSpatialHashResolution, kAngularHashResolution, 0.9}}(particles);

  auto per_cluster_estimates =
      estimate_clusters(beluga::views::states(particles), beluga::views::weights(particles), clusters);

  // check that the number of clusters is correct
  ASSERT_EQ(per_cluster_estimates.size(), 4);

  // order by decreasing weight
  ranges::sort(per_cluster_estimates, std::less{}, [](const auto& e) { return e.weight; });

  // check that the cluster means were found in the expected order
  EXPECT_THAT(per_cluster_estimates[0].mean, SE2Near(SO2d{0.0}, Vector2d{9.0, 9.0}, kTolerance));
  EXPECT_THAT(per_cluster_estimates[1].mean, SE2Near(SO2d{0.0}, Vector2d{27.0, 9.0}, kTolerance));
  EXPECT_THAT(per_cluster_estimates[2].mean, SE2Near(SO2d{0.0}, Vector2d{9.0, 27.0}, kTolerance));
  EXPECT_THAT(per_cluster_estimates[3].mean, SE2Near(SO2d{0.0}, Vector2d{27.0, 27.0}, kTolerance));
}

TEST_F(ClusterBasedEstimationDetailTesting, ClusterEstimation) {
  // test the weights have effect by selecting a few states and ignoring others
  const auto states = std::vector{
      SE2d{SO2d{Sophus::Constants<double>::pi() / 6}, Vector2d{0.0, -3.0}},  //
      SE2d{SO2d{Sophus::Constants<double>::pi() / 2}, Vector2d{1.0, -2.0}},  //
      SE2d{SO2d{Sophus::Constants<double>::pi() / 3}, Vector2d{2.0, -1.0}},  //
      SE2d{SO2d{Sophus::Constants<double>::pi() / 2}, Vector2d{1.0, -2.0}},  //
      SE2d{SO2d{Sophus::Constants<double>::pi() / 6}, Vector2d{2.0, -3.0}},  //
      SE2d{SO2d{Sophus::Constants<double>::pi() / 2}, Vector2d{3.0, -2.0}},  //
      SE2d{SO2d{Sophus::Constants<double>::pi() / 3}, Vector2d{4.0, -2.0}},  //
      SE2d{SO2d{Sophus::Constants<double>::pi() / 2}, Vector2d{0.0, -3.0}},  //
  };

  // cluster 0 has the max weight, except for cluster 3 which will be ignored because it has only a single particle
  const auto weights = std::vector{0.5, 0.5, 0.2, 0.3, 0.3, 0.2, 0.2, 1.0};
  const auto clusters = std::vector{0, 0, 1, 2, 2, 1, 1, 3};

  const auto particles = ranges::views::zip(states, weights, clusters);

  auto cluster_0_particles =
      particles | ranges::views::cache1 | ranges::views::filter([](const auto& p) { return std::get<2>(p) == 0; });

  auto cluster_0_states = cluster_0_particles | beluga::views::elements<0>;
  auto cluster_0_weights = cluster_0_particles | beluga::views::elements<1>;

  const auto [expected_pose, expected_covariance] = beluga::estimate(cluster_0_states, cluster_0_weights);
  const auto per_cluster_estimates = beluga::estimate_clusters(states, weights, clusters);

  ASSERT_EQ(per_cluster_estimates.size(), 3);  // cluster 3 should be ignored because it has only one particle

  const auto [_, pose, covariance] =
      *ranges::max_element(per_cluster_estimates, std::less{}, [](const auto& e) { return e.weight; });

  constexpr double kTolerance = 0.001;

  ASSERT_THAT(pose, SE2Near(expected_pose.so2(), expected_pose.translation(), kTolerance));
  ASSERT_THAT(covariance.col(0).eval(), Vector3Near(expected_covariance.col(0).eval(), kTolerance));
  ASSERT_THAT(covariance.col(1).eval(), Vector3Near(expected_covariance.col(1).eval(), kTolerance));
  ASSERT_THAT(covariance.col(2).eval(), Vector3Near(expected_covariance.col(2).eval(), kTolerance));
}

TEST_F(ClusterBasedEstimationDetailTesting, HeaviestClusterSelectionTest) {
  const auto particles = make_particle_multicluster_dataset(-2.0, +2.0, -2.0, +2.0, 0.025);

  // determine the expected values of the mean and covariance of the highest
  // weight cluster
  const auto max_peak_filter = [](const auto& s) { return s.translation().x() >= 0.0 && s.translation().y() >= 0.0; };
  const auto mask_filter = [](const auto& sample) { return std::get<1>(sample); };

  auto max_peak_mask = beluga::views::states(particles) | ranges::views::transform(max_peak_filter);
  auto max_peak_masked_states = ranges::views::zip(beluga::views::states(particles), max_peak_mask) |
                                ranges::views::filter(mask_filter) | beluga::views::elements<0>;
  auto max_peak_masked_weights = ranges::views::zip(beluga::views::weights(particles), max_peak_mask) |
                                 ranges::views::filter(mask_filter) | beluga::views::elements<0>;

  const auto [expected_pose, expected_covariance] = beluga::estimate(max_peak_masked_states, max_peak_masked_weights);

  const auto [pose, covariance] = beluga::cluster_based_estimate(particles);

  ASSERT_THAT(pose, SE2Near(expected_pose.so2(), expected_pose.translation(), kTolerance));
  ASSERT_NEAR(covariance(0, 0), expected_covariance(0, 0), 0.001);
  ASSERT_NEAR(covariance(0, 1), expected_covariance(0, 1), 0.001);
  ASSERT_NEAR(covariance(0, 2), expected_covariance(0, 2), 0.001);
  ASSERT_NEAR(covariance(1, 0), expected_covariance(1, 0), 0.001);
  ASSERT_NEAR(covariance(1, 1), expected_covariance(1, 1), 0.001);
  ASSERT_NEAR(covariance(1, 2), expected_covariance(1, 2), 0.001);
  ASSERT_NEAR(covariance(2, 0), expected_covariance(2, 0), 0.001);
  ASSERT_NEAR(covariance(2, 1), expected_covariance(2, 1), 0.001);
  ASSERT_NEAR(covariance(2, 2), expected_covariance(2, 2), 0.001);
}

TEST_F(ClusterBasedEstimationDetailTesting, NightmareDistributionTest) {
  // particles so far away that they are isolated and will therefore form four separate single
  // particle clusters
  const auto states = std::vector{
      SE2d{SO2d{0.0}, Vector2d{-10.0, -10.0}},  //
      SE2d{SO2d{0.0}, Vector2d{-10.0, +10.0}},  //
      SE2d{SO2d{0.0}, Vector2d{+10.0, -10.0}},  //
      SE2d{SO2d{0.0}, Vector2d{+10.0, +10.0}}};
  const auto weights = std::vector<beluga::Weight>{0.2, 0.2, 0.2, 0.2};

  // in this case, the cluster algorithm will not be able to group the particles and will
  // default to the set mean and covariance
  const auto [expected_pose, expected_covariance] = beluga::estimate(states, weights);

  auto particles = ranges::views::zip(states, weights) | ranges::to<std::vector>();
  const auto [pose, covariance] = beluga::cluster_based_estimate(particles);

  ASSERT_THAT(pose, SE2Near(expected_pose.so2(), expected_pose.translation(), kTolerance));
  ASSERT_NEAR(covariance(0, 0), expected_covariance(0, 0), 0.001);
  ASSERT_NEAR(covariance(0, 1), expected_covariance(0, 1), 0.001);
  ASSERT_NEAR(covariance(0, 2), expected_covariance(0, 2), 0.001);
  ASSERT_NEAR(covariance(1, 0), expected_covariance(1, 0), 0.001);
  ASSERT_NEAR(covariance(1, 1), expected_covariance(1, 1), 0.001);
  ASSERT_NEAR(covariance(1, 2), expected_covariance(1, 2), 0.001);
  ASSERT_NEAR(covariance(2, 0), expected_covariance(2, 0), 0.001);
  ASSERT_NEAR(covariance(2, 1), expected_covariance(2, 1), 0.001);
  ASSERT_NEAR(covariance(2, 2), expected_covariance(2, 2), 0.001);
}

}  // namespace

}  // namespace beluga

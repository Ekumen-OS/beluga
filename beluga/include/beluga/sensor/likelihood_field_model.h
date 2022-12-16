// Copyright 2022 Ekumen, Inc.
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

#include <beluga/algorithm/distance_map.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <shared_mutex>
#include <vector>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace beluga {

struct LikelihoodFieldModelParam {
  double max_obstacle_distance;
  double min_laser_distance;
  double max_laser_distance;
  double z_hit;
  double z_random;
  double sigma_hit;
};

template <class Mixin, class OccupancyGrid>
class LikelihoodFieldModel : public Mixin {
 public:
  template <class... Args>
  explicit LikelihoodFieldModel(const LikelihoodFieldModelParam& params, const OccupancyGrid& grid, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...),
        grid_{grid},
        free_cells_{make_free_cells_vector(grid)},
        likelihood_field_{make_likelihood_field(params, grid)} {}

  const auto& likelihood_field() const { return likelihood_field_; }

  template <class Generator>
  [[nodiscard]] Sophus::SE2d generate_random_state(Generator& generator) const {
    auto index_distribution = std::uniform_int_distribution<std::size_t>{0, free_cells_.size() - 1};
    return Sophus::SE2d{
        Sophus::SO2d::sampleUniform(generator),
        grid_.origin() * grid_.point(free_cells_[index_distribution(generator)])};
  }

  [[nodiscard]] double importance_weight(const Sophus::SE2d& state) const {
    const auto transform = grid_.origin().inverse() * state;
    const auto lock = std::shared_lock<std::shared_mutex>{points_mutex_};
    return std::transform_reduce(
        points_.cbegin(), points_.cend(), 0.0, std::plus{},
        [this, x_offset = transform.translation().x(), y_offset = transform.translation().y(),
         cos_theta = transform.so2().unit_complex().x(),
         sin_theta = transform.so2().unit_complex().y()](const auto& point) {
          // Transform the end point of the laser to the global coordinate system.
          // Not using Eigen/Sophus because they make the routine x10 slower.
          // See `benchmark_likelihood_field_model.cpp` for reference.
          const auto x = point.first * cos_theta - point.second * sin_theta + x_offset;
          const auto y = point.first * sin_theta + point.second * cos_theta + y_offset;
          const auto index = grid_.index(x, y);
          return index < likelihood_field_.size() ? likelihood_field_[index] : 0.0;
        });
  }

  void update_sensor(std::vector<std::pair<double, double>> points) {
    const auto lock = std::lock_guard<std::shared_mutex>{points_mutex_};
    points_ = std::move(points);
  }

 private:
  OccupancyGrid grid_;
  std::vector<std::size_t> free_cells_;
  std::vector<double> likelihood_field_;
  std::vector<std::pair<double, double>> points_;
  mutable std::shared_mutex points_mutex_;

  static std::vector<std::size_t> make_free_cells_vector(const OccupancyGrid& grid) {
    auto free_cells = std::vector<std::size_t>{};
    free_cells.reserve(grid.size());
    for (std::size_t index = 0; index < grid.size(); ++index) {
      if (OccupancyGrid::Traits::is_free(grid.data()[index])) {
        free_cells.push_back(index);
      }
    }
    return free_cells;
  }

  static std::vector<double> make_likelihood_field(const LikelihoodFieldModelParam& params, const OccupancyGrid& grid) {
    const auto obstacle_map = grid.data() | ranges::views::transform(&OccupancyGrid::Traits::is_occupied);

    auto squared_distance = [&grid, squared_max_distance = params.max_obstacle_distance * params.max_obstacle_distance](
                                std::size_t first, std::size_t second) {
      return std::min((grid.point(first) - grid.point(second)).squaredNorm(), squared_max_distance);
    };

    const auto distance_map = nearest_obstacle_distance_map(
        obstacle_map, squared_distance, std::bind(&OccupancyGrid::neighbors, &grid, std::placeholders::_1));

    auto to_likelihood = [amplitude =
                              params.z_hit / (params.sigma_hit * std::sqrt(2 * Sophus::Constants<double>::pi())),
                          two_squared_sigma = 2 * params.sigma_hit * params.sigma_hit,
                          offset = params.z_random / params.max_laser_distance](double squared_distance) {
      return amplitude * std::exp(-squared_distance / two_squared_sigma) + offset;
    };

    return distance_map | ranges::views::transform(to_likelihood) | ranges::to<std::vector>;
  }
};

}  // namespace beluga

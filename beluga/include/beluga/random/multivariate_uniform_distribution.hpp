// Copyright 2024 Ekumen, Inc.
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

#ifndef BELUGA_RANDOM_MULTIVARIATE_UNIFORM_DISTRIBUTION_HPP
#define BELUGA_RANDOM_MULTIVARIATE_UNIFORM_DISTRIBUTION_HPP

#include <random>

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

#include <Eigen/Geometry>

#include <beluga/sensor/data/occupancy_grid.hpp>

/**
 * \file
 * \brief Implementation of a multivariate uniform distribution for SE(2) and SE(3) spaces.
 */

namespace beluga {

/// Primary template for a multivariate uniform distribution.
/**
 * \tparam T The result type for sampling from the distribution.
 * \tparam Constraint The constraint type to limit the range of the distribution function.
 */
template <class T, class Constraint>
class MultivariateUniformDistribution;

/// Specialization of multivariate uniform distribution for bounding regions in 2D space.
template <>
class MultivariateUniformDistribution<Sophus::SE2d, Eigen::AlignedBox2d> {
 public:
  /// Constructs a multivariate uniform distribution in SE(2) with 2D bounding region.
  /**
   * \param box The axis-aligned bounding region.
   */
  explicit MultivariateUniformDistribution(const Eigen::AlignedBox2d& box)
      : x_distribution_{box.min().x(), box.max().x()}, y_distribution_{box.min().y(), box.max().y()} {}

  /// Generates a random 2D pose within the bounding region.
  /**
   * \tparam URNG The type of the random number generator.
   * \param engine The random number generator engine.
   * \return A random Sophus::SE2d pose.
   */
  template <class URNG>
  [[nodiscard]] Sophus::SE2d operator()(URNG& engine) {
    return Sophus::SE2d{
        Sophus::SO2d::sampleUniform(engine),
        Sophus::Vector2d{
            x_distribution_(engine),
            y_distribution_(engine),
        },
    };
  }

 private:
  std::uniform_real_distribution<double> x_distribution_;
  std::uniform_real_distribution<double> y_distribution_;
};

/// Deduction guide for bounding regions in SE2 space.
MultivariateUniformDistribution(const Eigen::AlignedBox2d&)
    -> MultivariateUniformDistribution<Sophus::SE2d, Eigen::AlignedBox2d>;

/// Specialization of multivariate uniform distribution for bounding regions in 3D space.
template <>
class MultivariateUniformDistribution<Sophus::SE3d, Eigen::AlignedBox3d> {
 public:
  /// Constructs a multivariate uniform distribution in SE(3) with 3D bounding region.
  /**
   * \param box The axis-aligned bounding region.
   */
  explicit MultivariateUniformDistribution(const Eigen::AlignedBox3d& box)
      : x_distribution_{box.min().x(), box.max().x()},
        y_distribution_{box.min().y(), box.max().y()},
        z_distribution_{box.min().z(), box.max().z()} {}

  /// Generates a random 3D pose within the bounding region.
  /**
   * \tparam URNG The type of the random number generator.
   * \param engine The random number generator engine.
   * \return A random Sophus::SE3d pose.
   */
  template <class URNG>
  [[nodiscard]] Sophus::SE3d operator()(URNG& engine) {
    return Sophus::SE3d{
        Sophus::SO3d::sampleUniform(engine),
        Sophus::Vector3d{
            x_distribution_(engine),
            y_distribution_(engine),
            z_distribution_(engine),
        },
    };
  }

 private:
  std::uniform_real_distribution<double> x_distribution_;
  std::uniform_real_distribution<double> y_distribution_;
  std::uniform_real_distribution<double> z_distribution_;
};

/// Deduction guide for bounding regions in SE3 space.
MultivariateUniformDistribution(const Eigen::AlignedBox3d&)
    -> MultivariateUniformDistribution<Sophus::SE3d, Eigen::AlignedBox3d>;

/// Specialization of multivariate uniform distribution for occupancy grids.
/**
 * The range of the distribution is limited to the free space available in the occupancy grid.
 * The rotation is sampled uniformly and the translation will match the exact grid coordinates
 * of one of the free cells.
 */
template <class OccupancyGrid>
class MultivariateUniformDistribution<Sophus::SE2d, OccupancyGrid> {
 public:
  /// Constructs a multivariate uniform distribution based on the provided occupancy grid.
  /**
   * \tparam OccupancyGrid A type of the occupancy grid.
   * \param grid The occupancy grid from which free states will be computed.
   */
  constexpr explicit MultivariateUniformDistribution(const OccupancyGrid& grid)
      : free_states_{compute_free_states(grid)}, distribution_{0, free_states_.size() - 1} {
    assert(!free_states_.empty());
  }

  /// Generates a random 2D pose.
  /**
   * This function generates a random pose by sampling a random rotation
   * from SO2 space and a random translation from the precomputed free states
   * based on the provided occupancy grid.
   *
   * \tparam URNG The type of the random number generator.
   * \param engine The random number generator engine.
   * \return A random Sophus::SE2d pose.
   */
  template <class URNG>
  [[nodiscard]] Sophus::SE2d operator()(URNG& engine) {
    return {Sophus::SO2d::sampleUniform(engine), free_states_[distribution_(engine)]};
  }

 private:
  std::vector<Eigen::Vector2d> free_states_;                 ///< Vector containing free states.
  std::uniform_int_distribution<std::size_t> distribution_;  ///< Uniform distribution for indices.

  static std::vector<Eigen::Vector2d> compute_free_states(const OccupancyGrid& grid) {
    return grid.coordinates_for(grid.free_cells(), OccupancyGrid::Frame::kGlobal) | ranges::to<std::vector>;
  }
};

/// Deduction guide for 2D occupancy grids.
template <class Derived>
MultivariateUniformDistribution(const BaseOccupancyGrid2<Derived>&)
    -> MultivariateUniformDistribution<Sophus::SE2d, Derived>;

}  // namespace beluga

#endif

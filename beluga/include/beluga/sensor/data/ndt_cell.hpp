
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

#ifndef BELUGA_SENSOR_DATA_NDT_CELL_HPP
#define BELUGA_SENSOR_DATA_NDT_CELL_HPP

#include <ostream>
#include <type_traits>

#include <beluga/sensor/data/sparse_value_grid.hpp>

#include <Eigen/Core>

#include <Eigen/src/Core/util/Constants.h>
#include <range/v3/view/zip.hpp>
#include <sophus/common.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>

namespace beluga {

/// Representation for a cell of a N dimensional NDT cell.
template <int NDim, typename Scalar = double>
struct NDTCell {
  static_assert(std::is_floating_point_v<Scalar>, "Scalar template parameter should be a floating point.");
  /// Number of dimensions of the cell's translation.
  static constexpr int num_dim = NDim;
  /// Floating point scalar type.
  using scalar_type = Scalar;
  /// Mean of the N dimensional normal distribution.
  Eigen::Matrix<Scalar, NDim, 1> mean;
  /// Covariance of the N dimensional normal distribution.
  Eigen::Matrix<Scalar, NDim, NDim> covariance;

  /// Get the L2 likelihood at measurement, scaled by d1 and d2. It assumes the measurement is pre-transformed
  /// into the same frame as this cell instance.
  [[nodiscard]] double likelihood_at(const NDTCell& measurement, double d1 = 1.0, double d2 = 1.0) const {
    const Eigen::Matrix<Scalar, NDim, 1> error = measurement.mean - mean;
    const double rhs =
        std::exp((-d2 / 2.0) * error.transpose() * (measurement.covariance + covariance).inverse() * error);
    return d1 * rhs;
  }

  /// Ostream overload mostly for debugging purposes.
  friend inline std::ostream& operator<<(std::ostream& os, const NDTCell& cell) {
    os << "Mean \n" << cell.mean.transpose() << " \n\nCovariance: \n" << cell.covariance;
    return os;
  }

  /// Transform the normal distribution according to tf, both mean and covariance.
  friend inline NDTCell operator*(const Sophus::SE2<scalar_type>& tf, const NDTCell& ndt_cell) {
    static_assert(num_dim == 2, "Cannot transform a non 2D NDT Cell with a SE2 transform.");
    const Eigen::Vector2d uij = tf * ndt_cell.mean;
    const Eigen::Matrix2Xd cov = tf.so2().matrix() * ndt_cell.covariance * tf.so2().matrix().transpose();
    return NDTCell{uij, cov};
  }

  /// Transform the normal distribution according to tf, both mean and covariance.
  friend inline NDTCell operator*(const Sophus::SE3<scalar_type>& tf, const NDTCell& ndt_cell) {
    static_assert(num_dim == 3, "Cannot transform a non 3D NDT Cell with a SE3 transform.");
    const Eigen::Vector3d uij = tf * ndt_cell.mean;
    const Eigen::Matrix3Xd cov = tf.so2().matrix() * ndt_cell.covariance * tf.so2().matrix().transpose();
    return NDTCell{uij, cov};
  }
};

/// Convenience alias for a 2D NDT cell with double representation.
using NDTCell2d = NDTCell<2, double>;
/// Convenience alias for a 2D NDT cell with float representation.
using NDTCell2f = NDTCell<2, float>;
/// Convenience alias for a 3D NDT cell with double representation.
using NDTCell3d = NDTCell<3, double>;
/// Convenience alias for a 3D NDT cell with float representation.
using NDTCell3f = NDTCell<3, float>;

}  // namespace beluga

#endif

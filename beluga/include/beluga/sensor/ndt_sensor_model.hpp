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

#ifndef BELUGA_SENSOR_NDT_SENSOR_MODEL_HPP
#define BELUGA_SENSOR_NDT_SENSOR_MODEL_HPP

#include <cassert>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <H5Cpp.h>

#include <Eigen/Core>

#include <sophus/common.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>

#include <beluga/sensor/data/ndt_cell.hpp>
#include <beluga/sensor/data/sparse_value_grid.hpp>

namespace beluga {

namespace detail {
/// Hash function for N dimensional Eigen::Vectors of int.
/// The code is from `hash_combine` function of the Boost library. See
/// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
template <int N>
struct CellHasher {
  /// Hashes an integer N dimensional integer vector.
  size_t operator()(const Eigen::Matrix<int, N, 1> vector) const {
    size_t seed = 0;
    for (auto i = 0L; i < vector.size(); ++i) {
      auto elem = *(vector.data() + i);
      seed ^= std::hash<int>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

/// Fit a vector of points to an NDT cell, by computing its mean and covariance.
template <int NDim, typename Scalar = double>
inline NDTCell<NDim, Scalar> fit_points(const std::vector<Eigen::Matrix<Scalar, NDim, 1>>& points) {
  static constexpr double kMinVariance = 1e-5;
  Eigen::Map<const Eigen::Matrix<Scalar, NDim, Eigen::Dynamic>> points_view(
      reinterpret_cast<const Scalar*>(points.data()), NDim, static_cast<int64_t>(points.size()));
  const Eigen::Matrix<Scalar, NDim, 1> mean = points_view.rowwise().mean();
  const Eigen::Matrix<Scalar, NDim, Eigen::Dynamic> centered = points_view.colwise() - mean;
  // Use sample covariance.
  Eigen::Matrix<Scalar, NDim, Eigen::Dynamic> cov =
      (centered * centered.transpose()) / static_cast<double>(points_view.cols() - 1);
  cov(0, 0) = std::max(cov(0, 0), kMinVariance);
  cov(1, 1) = std::max(cov(1, 1), kMinVariance);
  if constexpr (NDim == 3) {
    cov(2, 2) = std::max(cov(2, 2), kMinVariance);
  }
  return NDTCell<NDim, Scalar>{mean, cov};
}

/// Given a number of N dimensional points and a resolution, constructs a vector of NDT cells, by clusterizing the
/// points using 'resolution' and fitting a normal distribution to each of the resulting clusters if they contain a
/// minimum number of points in them.
template <int NDim, typename Scalar = double>
inline std::vector<NDTCell<NDim, Scalar>> to_cells(
    const std::vector<Eigen::Matrix<Scalar, NDim, 1>>& points,
    const double resolution) {
  static constexpr int kMinPointsPerCell = 5;

  const Eigen::Map<const Eigen::Matrix<Scalar, NDim, Eigen::Dynamic>> points_view(
      reinterpret_cast<const Scalar*>(points.data()), NDim, static_cast<int64_t>(points.size()));

  std::vector<NDTCell<NDim, Scalar>> ret;
  ret.reserve(static_cast<size_t>(points_view.cols()) / kMinPointsPerCell);

  std::unordered_map<Eigen::Matrix<int, NDim, 1>, std::vector<Eigen::Matrix<Scalar, NDim, 1>>, CellHasher<NDim>>
      cell_grid;
  for (const Eigen::Matrix<Scalar, NDim, 1>& col : points) {
    cell_grid[(col / resolution).template cast<int>()].emplace_back(col);
  }

  for (const auto& [cell, points_in_cell] : cell_grid) {
    if (points_in_cell.size() < kMinPointsPerCell) {
      continue;
    }
    ret.push_back(fit_points<NDim, Scalar>(points_in_cell));
  }

  return ret;
}
/// Default neighbor kernel for the 2D NDT sensor model.
const std::vector<Eigen::Vector2i> kDefaultNeighborKernel2d = {
    Eigen::Vector2i{-1, -1}, Eigen::Vector2i{-1, -0}, Eigen::Vector2i{-1, 1},
    Eigen::Vector2i{0, -1},  Eigen::Vector2i{0, 0},   Eigen::Vector2i{0, 1},
    Eigen::Vector2i{1, -1},  Eigen::Vector2i{1, 0},   Eigen::Vector2i{1, 1},
};

/// Default neighbor kernel for the 3D NDT sensor model.
const std::vector<Eigen::Vector3i> kDefaultNeighborKernel3d = {
    // TODO(Ramiro) revisit this kernel when we implement the 3D sensor model and extend this if it's computationally
    // feasible.
    Eigen::Vector3i{0, 0, 0},  Eigen::Vector3i{0, 0, 1},  Eigen::Vector3i{0, 0, -1}, Eigen::Vector3i{0, 1, 0},
    Eigen::Vector3i{0, -1, 0}, Eigen::Vector3i{-1, 0, 0}, Eigen::Vector3i{1, 0, 0},
};

/// Helper to get the default neighbors kernel
template <int NDim>
constexpr std::conditional_t<NDim == 2, std::vector<Eigen::Vector2i>, std::vector<Eigen::Vector3i>>
get_default_neighbors_kernel() {
  if constexpr (NDim == 2) {
    return kDefaultNeighborKernel2d;
  } else {
    return kDefaultNeighborKernel3d;
  }
}

}  // namespace detail

/// Parameters used to construct a NDTSensorModel instance.
template <int NDim>
struct NDTModelParam {
  static_assert(NDim == 2 or NDim == 3, "Only 2D or 3D is supported for the NDT sensor model.");
  /// Likelihood for measurements that lie inside cells that are not present in the map.
  double minimum_likelihood = 0;
  /// Scaling parameter d1 in literature, used for scaling 2D likelihoods.
  double d1 = 1.0;
  /// Scaling parameter d2 in literature, used for scaling 2D likelihoods.
  double d2 = 1.0;
  /// Neighbor kernel used for likelihood computation.
  std::conditional_t<NDim == 2, std::vector<Eigen::Vector2i>, std::vector<Eigen::Vector3i>> neighbors_kernel =
      detail::get_default_neighbors_kernel<NDim>();
};

/// Convenience alias for a 2d parameters struct for the NDT sensor model.
using NDTModelParam2d = NDTModelParam<2>;

/// Convenience alias for a 3d parameters struct for the NDT sensor model.
using NDTModelParam3d = NDTModelParam<3>;
/// NDT sensor model for range finders.
/**
 * This class satisfies \ref SensorModelPage.
 *
 * \tparam SparseGridT Type representing a sparse NDT grid, as a specialization of 'beluga::SparseValueGrid'.
 */
template <typename SparseGridT>
class NDTSensorModel {
 public:
  // static_assert(std::is_same_v<SparseValueGrid<typename SparseGridT::map_type>, SparseGridT>);
  /// NDT Cell type.
  using ndt_cell_type = typename SparseGridT::mapped_type;
  static_assert(ndt_cell_type::num_dim == 2, "NDT sensor model is only implemented for 2D problems.");
  /// State type of a particle.
  using state_type = std::conditional_t<
      ndt_cell_type::num_dim == 2,
      Sophus::SE2<typename ndt_cell_type::scalar_type>,
      Sophus::SE3<typename ndt_cell_type::scalar_type>>;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: a point cloud for the range finder.
  using measurement_type = std::vector<Eigen::Matrix<typename ndt_cell_type::scalar_type, ndt_cell_type::num_dim, 1>>;
  /// Map representation type.
  using map_type = SparseGridT;
  /// Parameter type that the constructor uses to configure the NDT sensor model.
  using param_type = NDTModelParam<ndt_cell_type::num_dim>;

  /// Constructs a NDTSensorModel instance.
  /**
   * \param params Parameters to configure this instance.
   *  See beluga::NDTModelParam for details.
   * \param cells_data Sparse grid representing an NDT map, used for calculating importance weights for the different
   * particles.
   */
  NDTSensorModel(param_type params, SparseGridT cells_data)
      : params_{std::move(params)}, cells_data_{std::move(cells_data)} {
    assert(params_.minimum_likelihood >= 0);
  }

  /// Returns a state weighting function conditioned on 2D / 3D lidar hits.
  /**
   * \param points 2D lidar hit points in the reference frame of particle states.
   * \return a state weighting function satisfying \ref StateWeightingFunctionPage
   *  and borrowing a reference to this sensor model (and thus their lifetime are bound).
   */
  [[nodiscard]] auto operator()(measurement_type&& points) const {
    return [this, cells = detail::to_cells<ndt_cell_type::num_dim, typename ndt_cell_type::scalar_type>(
                      points, cells_data_.resolution())](const state_type& state) -> weight_type {
      return std::transform_reduce(
          cells.cbegin(), cells.cend(), 1.0, std::plus{},
          [this, state](const ndt_cell_type& ndt_cell) { return likelihood_at(state * ndt_cell); });
    };
  }

  /// Returns the L2 likelihood scaled by 'd1' and 'd2' set in the parameters for this instance for 'measurement', for
  /// the 3x3 cells around the measurement cell, or 'params_.min_likelihood', whichever is higher.
  [[nodiscard]] double likelihood_at(const ndt_cell_type& measurement) const {
    double likelihood = 0;
    const typename map_type::key_type measurement_cell = cells_data_.cell_near(measurement.mean);
    for (const auto& offset : params_.neighbors_kernel) {
      const auto maybe_ndt = cells_data_.data_at(measurement_cell + offset);
      if (maybe_ndt.has_value()) {
        likelihood += maybe_ndt->likelihood_at(measurement, params_.d1, params_.d2);
      }
    }
    return std::max(likelihood, params_.minimum_likelihood);
  }

 private:
  const param_type params_;
  const SparseGridT cells_data_;
};

namespace io {

/// Loads a 2D NDT map representation from a hdf5 file, with the following datasets:
/// - "resolution": () resolution for the discrete grid (cells are resolution x
///   resolution m^2 squares).
/// - "cells": (NUM_CELLS, 2) that contains the cell coordinates.
/// - "means": (NUM_CELLS, 2) that contains the 2d mean of the normal distribution
///   of the cell.
/// - "covariances": (NUM_CELLS, 2, 2) Contains the covariance for each cell.
///
///  \tparam NDTMapRepresentation A specialized SparseValueGrid (see sensor/data/sparse_value_grid.hpp), where
///  mapped_type == NDTCell2d, that will represent the NDT map as a mapping from 2D cells to NDTCells.
template <typename NDTMapRepresentationT>
NDTMapRepresentationT load_from_hdf5_2d(const std::filesystem::path& path_to_hdf5_file) {
  static_assert(std::is_same_v<typename NDTMapRepresentationT::mapped_type, NDTCell2d>);
  static_assert(std::is_same_v<typename NDTMapRepresentationT::key_type, Eigen::Vector2i>);
  if (!std::filesystem::exists(path_to_hdf5_file) || std::filesystem::is_directory(path_to_hdf5_file)) {
    std::stringstream ss;
    ss << "Couldn't find a valid HDF5 file at " << path_to_hdf5_file;
    throw std::invalid_argument(ss.str());
  }

  const H5::H5File file(path_to_hdf5_file, H5F_ACC_RDONLY);
  const H5::DataSet means_dataset = file.openDataSet("means");
  const H5::DataSet covariances_dataset = file.openDataSet("covariances");
  const H5::DataSet resolution_dataset = file.openDataSet("resolution");
  const H5::DataSet cells_dataset = file.openDataSet("cells");

  std::array<hsize_t, 2> dims_out;
  means_dataset.getSpace().getSimpleExtentDims(dims_out.data(), nullptr);

  // The static cast is necessary because hsize_t and size_t are not the same in all platforms.
  const std::array<std::size_t, 2> dims{
      static_cast<std::size_t>(dims_out[0]),
      static_cast<std::size_t>(dims_out[1]),
  };

  Eigen::Matrix2Xd means_matrix(dims[1], dims[0]);
  Eigen::Matrix2Xi cells_matrix(dims[1], dims[0]);

  means_dataset.read(means_matrix.data(), H5::PredType::NATIVE_DOUBLE);
  cells_dataset.read(cells_matrix.data(), H5::PredType::NATIVE_INT);

  std::vector<Eigen::Array<double, 2, 2>> covariances(dims[0]);
  covariances_dataset.read(covariances.data(), H5::PredType::NATIVE_DOUBLE);

  double resolution;

  resolution_dataset.read(&resolution, H5::PredType::NATIVE_DOUBLE);

  typename NDTMapRepresentationT::map_type map{};

  // Note: Ranges::zip_view doesn't seem to work in old Eigen.
  for (size_t i = 0; i < covariances.size(); ++i) {
    map[cells_matrix.col(static_cast<Eigen::Index>(i))] =
        NDTCell2d{means_matrix.col(static_cast<Eigen::Index>(i)), covariances.at(i)};
  }

  return NDTMapRepresentationT{std::move(map), resolution};
}

}  // namespace io

}  // namespace beluga

#endif

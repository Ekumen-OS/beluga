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

#pragma once

#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/Core>
#include <beluga/sensor/data/ndt_cell.hpp>
#include <beluga/sensor/data/sparse_value_grid.hpp>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <range/v3/view/zip.hpp>
#include <sophus/common.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include "H5Cpp.h"

namespace beluga {

namespace detail {
/// Hash function for N dimensional Eigen::Vectors of int.
/// The code is from `hash_combine` function of the Boost library. See
/// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
template <size_t N>
struct CellHasher {
  /// Hashes an integer N dimensional integer vector.
  size_t operator()(const Eigen::Vector<int, N> vector) const {
    size_t seed = 0;
    for (auto i = 0L; i < vector.size(); ++i) {
      auto elem = *(vector.data() + i);
      seed ^= std::hash<int>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

/// Fit a vector of points to an NDT cell, by computing its mean and covariance.
template <size_t NDim, typename Scalar = double>
inline NDTCell<NDim, Scalar> fit_points(const std::vector<Eigen::Vector<Scalar, NDim>>& points) {
  static constexpr double kMinVariance = 1e-5;
  Eigen::Map<const Eigen::Matrix<Scalar, NDim, Eigen::Dynamic>> points_view(
      reinterpret_cast<const Scalar*>(points.data()), 2, static_cast<int64_t>(points.size()));
  const Eigen::Vector<Scalar, NDim> mean = points_view.rowwise().mean();
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
template <size_t NDim, typename Scalar = double>
inline std::vector<NDTCell<NDim, Scalar>> to_cells(
    const std::vector<Eigen::Vector<Scalar, NDim>>& points,
    const double resolution) {
  static constexpr int kMinPointsPerCell = 5;

  Eigen::Map<const Eigen::Matrix<Scalar, NDim, Eigen::Dynamic>> points_view(
      reinterpret_cast<const Scalar*>(points.data()), NDim, static_cast<int64_t>(points.size()));

  std::vector<NDTCell<NDim, Scalar>> ret;
  ret.reserve(static_cast<size_t>(points_view.cols()) / kMinPointsPerCell);

  std::unordered_map<Eigen::Vector<int, NDim>, std::vector<Eigen::Vector<Scalar, NDim>>, CellHasher<NDim>> cell_grid;
  for (const Eigen::Vector2d& col : points) {
    cell_grid[(col / resolution).cast<int>()].emplace_back(col);
  }

  for (const auto& [cell, points_in_cell] : cell_grid) {
    if (points_in_cell.size() < kMinPointsPerCell) {
      continue;
    }
    ret.push_back(fit_points<NDim, Scalar>(points_in_cell));
  }

  return ret;
}

}  // namespace detail

/// Parameters used to construct a NDTSensorModel instance.
struct NDTModelParam {
  /// Likelihood for measurements that lie inside cells that are not present in the map.
  double minimum_likelihood = 0;
  /// Scaling parameter d1 in literature, used for scaling 2D likelihoods.
  double d1 = 1.0;
  /// Scaling parameter d2 in literature, used for scaling 2D likelihoods.
  double d2 = 1.0;
};

/// NDT sensor model for range finders.
/**
 * This class satisfies \ref SensorModelPage.
 *
 * \tparam SparseGridT Type representing a sparse NDT grid, as a specialization of 'beluga::SparseValueGrid'.
 */
template <typename SparseGridT>
class NDTSensorModel {
 public:
  static_assert(std::is_same_v<SparseValueGrid<typename SparseGridT::map_type>, SparseGridT>);
  /// NDT Cell type.
  using ndt_cell_type = typename SparseGridT::mapped_type;
  static_assert(
      ndt_cell_type::num_dim == 2 || ndt_cell_type::num_dim == 3,
      "NDT sensor model is only implemented for 2D or 3D problems.");
  /// State type of a particle.
  using state_type = std::conditional_t<
      ndt_cell_type::num_dim == 2,
      Sophus::SE2<typename ndt_cell_type::scalar_type>,
      Sophus::SE3<typename ndt_cell_type::scalar_type>>;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: a point cloud for the range finder.
  using measurement_type = std::vector<Eigen::Vector<typename ndt_cell_type::scalar_type, ndt_cell_type::num_dim>>;
  /// Map representation type.
  using map_type = SparseGridT;
  /// Parameter type that the constructor uses to configure the NDT sensor model.
  using param_type = NDTModelParam;

  /// Constructs a NDTSensorModel instance.
  /**
   * \param params Parameters to configure this instance.
   *  See beluga::NDTModelParam for details.
   * \param cells_data Sparse grid representing an NDT map, used for calculating importance weights for the different
   * particles.
   */
  NDTSensorModel(param_type params, SparseGridT cells_data)
      : params_{std::move(params)}, cells_data_{std::move(cells_data)} {}

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

  /// Returns the L2 likelihood scaled by 'd1' and 'd2' set in the parameters for this instance for 'measurement, or
  /// 'params_.min_likelihood' if the cell corresponding to 'measurement' doesn't exist in the map.
  [[nodiscard]] double likelihood_at(const ndt_cell_type& measurement) const {
    const auto maybe_cell = cells_data_.data_near(measurement.mean);
    if (!maybe_cell.has_value()) {
      return params_.minimum_likelihood;
    }
    return maybe_cell->likelihood_at(measurement, params_.d1, params_.d2);
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

  H5::H5File file(path_to_hdf5_file, H5F_ACC_RDONLY);
  H5::DataSet means_dataset = file.openDataSet("means");
  H5::DataSet covariances_dataset = file.openDataSet("covariances");
  H5::DataSet resolution_dataset = file.openDataSet("resolution");
  H5::DataSet cells_dataset = file.openDataSet("cells");

  std::array<hsize_t, 2> dims_out;
  means_dataset.getSpace().getSimpleExtentDims(dims_out.data(), nullptr);

  Eigen::Matrix2Xd means_matrix(dims_out[1], dims_out[0]);
  Eigen::Matrix2Xi cells_matrix(dims_out[1], dims_out[0]);

  means_dataset.read(means_matrix.data(), H5::PredType::NATIVE_DOUBLE);
  cells_dataset.read(cells_matrix.data(), H5::PredType::NATIVE_INT);

  std::vector<Eigen::Array<double, 2, 2>> covariances(dims_out[0]);
  covariances_dataset.read(covariances.data(), H5::PredType::NATIVE_DOUBLE);

  double resolution;

  resolution_dataset.read(&resolution, H5::PredType::NATIVE_DOUBLE);

  typename NDTMapRepresentationT::map_type map{};

  for (const auto& [cell, mean, covariance] :
       ranges::zip_view(cells_matrix.colwise(), means_matrix.colwise(), covariances)) {
    map[cell] = NDTCell2d{mean, covariance};
  }
  return NDTMapRepresentationT{std::move(map), resolution};
}

}  // namespace io

}  // namespace beluga

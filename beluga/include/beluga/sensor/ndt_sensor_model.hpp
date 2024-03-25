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

#include <Eigen/Core>
#include <beluga/sensor/data/sparse_value_grid.hpp>
#include <filesystem>
#include <range/v3/view/zip.hpp>
#include <sophus/common.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include "H5Cpp.h"

namespace beluga {

// Hash function for Eigen::Vector2i.
// The code is from `hash_combine` function of the Boost library. See
// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
struct CellHasher {
  /// Hashes an integer 2D vector.
  size_t operator()(const Eigen::Vector2i& vector) const {
    size_t seed = 0;
    for (auto i = 0L; i < vector.size(); ++i) {
      auto elem = *(vector.data() + i);
      seed ^= std::hash<int>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

/// Parameters used to construct a NDTSebsorModel instance.
struct NDTModelParam {
  /// Likelihood for measurements that lie inside cells that are not present in the map.
  double minimum_likelihood = 0;
  /// Scaling parameter d1 in literature, used for scaling 2D likelihoods.
  double d1 = 1.0;
  /// Scaling parameter d2 in literature, used for scaling 2D likelihoods.
  double d2 = 1.0;
};

/// Representation for a cell of a 2D NDT map.
struct NDTCell {
  /// Mean of the 2D normal distribution.
  Eigen::Vector2d mean;
  /// Covariance of the 2D normal distribution.
  Eigen::Matrix<double, 2, 2> covariance;

  /// Get the L2 likelihood at measurement, scaled by d1 and d2.
  [[nodiscard]] double likelihood_at(const Eigen::Vector2d& measurement, double d1 = 1.0, double d2 = 1.0) const {
    const double constant = d1 / (2.0 * Sophus::Constants<double>::pi() * std::sqrt(covariance.determinant()));
    const Eigen::Vector2d error = measurement - mean;
    const double rhs = std::exp((-d2 / 2.0) * error.transpose() * covariance.inverse() * error);
    return constant * rhs;
  }
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
  static_assert(std::is_same_v<typename SparseGridT::mapped_type, NDTCell>);
  /// State type of a particle.
  using state_type = Sophus::SE2d;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: a point cloud for the range finder.
  using measurement_type = std::vector<std::pair<double, double>>;
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
  NDTSensorModel(param_type param, SparseGridT cells_data)
      : params_{std::move(param)}, cells_data_{std::move(cells_data)} {}

  /// Returns a state weighting function conditioned on 2D lidar hits.
  /**
   * \param points 2D lidar hit points in the reference frame of particle states.
   * \return a state weighting function satisfying \ref StateWeightingFunctionPage
   *  and borrowing a reference to this sensor model (and thus their lifetime are bound).
   */
  [[nodiscard]] auto operator()(measurement_type&& points) const {
    return [this, points = std::move(points)](const state_type& state) -> weight_type {
      const auto x_offset = state.translation().x();
      const auto y_offset = state.translation().y();
      const auto cos_theta = state.so2().unit_complex().x();
      const auto sin_theta = state.so2().unit_complex().y();
      return std::transform_reduce(
          points.cbegin(), points.cend(), 1.0, std::plus{},
          [this, x_offset, y_offset, cos_theta, sin_theta](const auto& point) {
            // Transform the end point of the laser to the grid local coordinate system.
            // Not using Eigen/Sophus because they make the routine x10 slower.
            // See `benchmark_likelihood_field_model.cpp` for reference.
            const auto x = point.first * cos_theta - point.second * sin_theta + x_offset;
            const auto y = point.first * sin_theta + point.second * cos_theta + y_offset;
            return likelihood_at({x, y});
          });
    };
  }

  /// Returns the L2 likelihood scaled by 'd1' and 'd2' set in the parameters for this instance for 'measurement, or
  /// 'params_.min_likelihood' if the cell corresponding to 'measurement' doesn't exist in the map.
  [[nodiscard]] double likelihood_at(const Eigen::Vector2d& measurement) const {
    const auto maybe_cell = cells_data_.data_near(measurement);
    if (!maybe_cell.has_value()) {
      return params_.minimum_likelihood;
    }
    return maybe_cell->likelihood_at(measurement, params_.d1, params_.d2);
  }

 private:
  const param_type params_;
  const SparseGridT cells_data_;
};

/// Loads a sparse grid from an hdf5 file, with the following structure:
// - "resolution": () resolution for the discrete grid (cells are resolution x
//   resolution m^2 squares).
// - "cells": (NUM_CELLS, 2) that contains the cell coordinates.
// - "means": (NUM_CELLS, 2) that contains the 2d mean of the normal distribution
//   of the cell.
// - "covariances": (NUM_CELLS, 2, 2) Contains the covariance for each cell.
template <typename SparseGridT>
SparseGridT load_from_hdf5(const std::filesystem::path& path_to_hdf5_file) {
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

  Eigen::MatrixX2d means_matrix(dims_out[0], dims_out[1]);
  Eigen::MatrixX2i cells_matrix(dims_out[0], dims_out[1]);

  means_dataset.read(means_matrix.data(), H5::PredType::NATIVE_DOUBLE);
  cells_dataset.read(cells_matrix.data(), H5::PredType::NATIVE_INT64);

  std::vector<Eigen::Array<double, 2, 2>> covariances(dims_out[0]);
  covariances_dataset.read(covariances.data(), H5::PredType::NATIVE_DOUBLE);

  double resolution;

  resolution_dataset.read(&resolution, H5::PredType::NATIVE_DOUBLE);
  typename SparseGridT::map_type map{};

  for (const auto& [cell, mean, covariance] :
       ranges::zip_view(cells_matrix.colwise(), means_matrix.colwise(), covariances)) {
    map[cell] = NDTCell{mean, covariance};
  }
  return SparseGridT{std::move(map), resolution};
}

}  // namespace beluga

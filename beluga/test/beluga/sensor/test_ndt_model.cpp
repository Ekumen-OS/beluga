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

#include <stdexcept>
#include <unordered_map>

#include <beluga/sensor/data/sparse_value_grid.hpp>
#include <beluga/sensor/ndt_sensor_model.hpp>

#include <Eigen/Core>

#include <gmock/gmock.h>
#include <gtest/gtest-typed-test.h>
#include <gtest/gtest.h>
#include <sophus/se2.hpp>

namespace beluga {

namespace {

Eigen::Matrix2Xd get_diagonal_covariance(double x_var = 0.5, double y_var = 0.5) {
  return Eigen::Vector2d{x_var, y_var}.asDiagonal();
}
using sparse_grid_2d_t = SparseValueGrid<std::unordered_map<Eigen::Vector2i, NDTCell2d, detail::CellHasher<2>>>;
}  // namespace

TEST(NDTSensorModelTests, CanConstruct) {
  NDTSensorModel{{}, sparse_grid_2d_t{}};
}

TEST(NDTSensorModelTests, MinLikelihood) {
  const double minimum_likelihood = 1e-6;

  NDTModelParam param{minimum_likelihood};
  NDTSensorModel model{param, sparse_grid_2d_t{}};

  for (const auto& val : {
           Eigen::Vector2d{0.1, 0.1},
           Eigen::Vector2d{0.15, 0.15},
           Eigen::Vector2d{0.25, 0.25},
           Eigen::Vector2d{0.35, 0.35},
           Eigen::Vector2d{0.45, 0.45},
           Eigen::Vector2d{0.50, 0.50},
           Eigen::Vector2d{1.65, 0.65},
           Eigen::Vector2d{0.75, 0.75},

       }) {
    ASSERT_DOUBLE_EQ(model.likelihood_at({val, get_diagonal_covariance()}), minimum_likelihood);
  }
}

TEST(NDTSensorModelTests, Likelihoood) {
  typename sparse_grid_2d_t::map_type map;
  {
    Eigen::Array<double, 2, 2> cov;
    // clang-format off
    cov << 0.5, 0.0,
           0.0, 0.3;
    // clang-format on
    Eigen::Vector2d mean(0.5, 0.5);
    map[Eigen::Vector2i(0, 0)] = NDTCell2d{mean, cov};
  }
  {
    Eigen::Array<double, 2, 2> cov;
    // clang-format off
    cov << 0.5, 0.0,
           0.0, 0.5;
    // clang-format on
    Eigen::Vector2d mean(1.5, 1.5);
    map[Eigen::Vector2i(1, 1)] = NDTCell2d{mean, cov};
  }
  sparse_grid_2d_t grid{std::move(map), 1.0};

  const double minimum_likelihood = -1.0;
  NDTModelParam param{minimum_likelihood};
  NDTSensorModel model{param, std::move(grid)};

  EXPECT_DOUBLE_EQ(model.likelihood_at({{0.5, 0.5}, get_diagonal_covariance()}), 1.3678794411714423);
  EXPECT_DOUBLE_EQ(model.likelihood_at({{0.8, 0.5}, get_diagonal_covariance()}), 1.4307317817730123);
  EXPECT_DOUBLE_EQ(model.likelihood_at({{0.5, 0.8}, get_diagonal_covariance()}), 1.4200370805919718);

  EXPECT_DOUBLE_EQ(model.likelihood_at({{1.5, 1.5}, get_diagonal_covariance()}), 1.3246524673583497);
  EXPECT_DOUBLE_EQ(model.likelihood_at({{1.8, 1.5}, get_diagonal_covariance()}), 1.1859229670198237);
  EXPECT_DOUBLE_EQ(model.likelihood_at({{1.5, 1.8}, get_diagonal_covariance()}), 1.1669230426687498);
}

TEST(NDTSensorModelTests, FitPoints) {
  {
    std::vector meas{
        Eigen::Vector2d{0.1, 0.2}, Eigen::Vector2d{0.1, 0.2}, Eigen::Vector2d{0.1, 0.2},
        Eigen::Vector2d{0.1, 0.2}, Eigen::Vector2d{0.1, 0.2}, Eigen::Vector2d{0.1, 0.2},
    };
    auto cell = detail::fit_points<2>(meas);
    ASSERT_TRUE(cell.mean.isApprox(Eigen::Vector2d{0.1, 0.2}));
    // We introduce a minimum variance to avoid numeric errors down the line.
    ASSERT_FALSE(cell.covariance.isZero());
  }
  {
    std::vector meas{
        Eigen::Vector2d{0.1, 0.2}, Eigen::Vector2d{0.1, 0.9}, Eigen::Vector2d{0.1, 0.2},
        Eigen::Vector2d{0.1, 0.9}, Eigen::Vector2d{0.1, 0.2}, Eigen::Vector2d{0.1, 0.2},
    };
    auto cell = detail::fit_points<2>(meas);
    ASSERT_TRUE(cell.mean.isApprox(Eigen::Vector2d{0.1, 0.433333}, 1e-6));
    ASSERT_FALSE(cell.covariance.isZero());
    ASSERT_GT(cell.covariance(1, 1), cell.covariance(0, 0));
  }
}

TEST(NDTSensorModelTests, ToCellsNotEnoughPointsInCell) {
  const double map_res = 0.5;
  std::vector map_data{
      Eigen::Vector2d{0.1, 0.2},
      Eigen::Vector2d{0.112, 0.22},
      Eigen::Vector2d{0.15, 0.23},
  };
  const auto cells = detail::to_cells<2UL>(map_data, map_res);
  ASSERT_EQ(cells.size(), 0UL);
}

TEST(NDTSensorModelTests, SensorModel) {
  double map_res = 0.5;
  std::vector map_data{
      Eigen::Vector2d{0.1, 0.2},  Eigen::Vector2d{0.112, 0.22}, Eigen::Vector2d{0.15, 0.23},
      Eigen::Vector2d{0.1, 0.24}, Eigen::Vector2d{0.16, 0.25},  Eigen::Vector2d{0.1, 0.26},
  };
  auto cells = detail::to_cells<2UL>(map_data, map_res);

  typename sparse_grid_2d_t::map_type map_cells_data;
  for (const auto& cell : cells) {
    map_cells_data[(cell.mean.array() / map_res).cast<int>()] = cell;
  }
  std::vector perfect_measurement = map_data;
  NDTSensorModel model{{}, sparse_grid_2d_t{map_cells_data, map_res}};
  auto state_weighing_fn = model(std::move(perfect_measurement));
  // This is a perfect hit, so we should expect weight to be 1 + num_cells == 2.
  ASSERT_DOUBLE_EQ(state_weighing_fn(Sophus::SE2d{}), 2);

  // Subtle miss, this value should be close to 1 but not quite.
  ASSERT_GT(state_weighing_fn(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{0.1, 0.1}}), 1.0);

  // This is a perfect miss, so we should expect weight to be 1.
  ASSERT_DOUBLE_EQ(state_weighing_fn(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{-10, -10}}), 1.0);
}

TEST(NDTSensorModelTests, LoadFromHDF5HappyPath) {
  const auto ndt_map_representation = io::load_from_hdf5_2d<sparse_grid_2d_t>("./test_data/turtlebot3_world.hdf5");
  ASSERT_EQ(ndt_map_representation.size(), 30UL);
}

TEST(NDTSensorModelTests, LoadFromHDF5NonExistingFile) {
  ASSERT_THROW(io::load_from_hdf5_2d<sparse_grid_2d_t>("bad_file.hdf5"), std::invalid_argument);
}

}  // namespace beluga

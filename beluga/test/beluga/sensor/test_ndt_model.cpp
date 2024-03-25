// Copyright 2023 Ekumen, Inc.
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
#include <gtest/gtest-typed-test.h>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <beluga/sensor/ndt_sensor_model.hpp>
#include <unordered_map>
#include "beluga/sensor/data/sparse_value_grid.hpp"
namespace beluga {

using sparse_grid_t = SparseValueGrid<std::unordered_map<Eigen::Vector2i, NDTCell, CellHasher>>;

TEST(NDTSensorModelTests, CanConstruct) {
  NDTSensorModel{{}, sparse_grid_t{}};
}

TEST(NDTSensorModelTests, MinLikelihood) {
  const double minimum_likelihood = -1.0;

  NDTModelParam param{minimum_likelihood};
  NDTSensorModel model{param, sparse_grid_t{}};

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
    ASSERT_DOUBLE_EQ(model.likelihood_at(val), minimum_likelihood);
  }
}

TEST(NDTSensorModelTests, Likelihoood) {
  typename sparse_grid_t::map_type map;
  {
    Eigen::Array<double, 2, 2> cov;
    // clang-format off
    cov << 0.5, 0.0,
           0.0, 0.3;
    // clang-format on
    Eigen::Vector2d mean(0.5, 0.5);
    map[Eigen::Vector2i(0, 0)] = NDTCell{mean, cov};
  }
  {
    Eigen::Array<double, 2, 2> cov;
    // clang-format off
    cov << 0.5, 0.0,
           0.0, 0.5;
    // clang-format on
    Eigen::Vector2d mean(1.5, 1.5);
    map[Eigen::Vector2i(1, 1)] = NDTCell{mean, cov};
  }
  sparse_grid_t grid{std::move(map), 1.0};

  const double minimum_likelihood = -1.0;
  NDTModelParam param{minimum_likelihood};
  NDTSensorModel model{param, std::move(grid)};

  EXPECT_DOUBLE_EQ(model.likelihood_at({0.5, 0.5}), 0.41093629604099985);
  EXPECT_DOUBLE_EQ(model.likelihood_at({0.8, 0.5}), 0.3755674961117193);
  EXPECT_DOUBLE_EQ(model.likelihood_at({0.5, 0.8}), 0.35369614780505748);

  EXPECT_DOUBLE_EQ(model.likelihood_at({1.5, 1.5}), 0.31830988618379069);
  EXPECT_DOUBLE_EQ(model.likelihood_at({1.8, 1.5}), 0.29091333156350158);
  EXPECT_DOUBLE_EQ(model.likelihood_at({1.5, 1.8}), 0.29091333156350158);
}

TEST(NDTCellTest, D1Scaling) {
  Eigen::Array<double, 2, 2> cov;
  // clang-format off
  cov << 0.5, 0.0,
          0.0, 0.5;
  // clang-format on
  Eigen::Vector2d mean{0.5, 0.5};
  NDTCell cell{mean, cov};

  const double d1 = 0.8;
  for (const auto& measurement : {
           Eigen::Vector2d{1.5, 1.2},
           Eigen::Vector2d{1.1, 1.2},
           Eigen::Vector2d{0.1, 0.7},
       }) {
    const auto unscaled_likelihood = cell.likelihood_at(measurement);
    const auto scaled_likelihood = cell.likelihood_at(measurement, d1);
    EXPECT_DOUBLE_EQ(unscaled_likelihood * d1, scaled_likelihood);
  }
}

TEST(NDTCellTest, D2Scaling) {
  Eigen::Array<double, 2, 2> cov;
  // clang-format off
  cov << 0.5, 0.0,
          0.0, 0.5;
  // clang-format on
  Eigen::Vector2d mean{0.5, 0.5};
  NDTCell cell{mean, cov};

  const double d2 = 0.8;
  EXPECT_DOUBLE_EQ(cell.likelihood_at(Eigen::Vector2d{1.5, 1.2}, 1.0, d2), 0.096643156212102871);
  EXPECT_DOUBLE_EQ(cell.likelihood_at(Eigen::Vector2d{0.12, 1.2}, 1.0, d2), 0.19161830396164195);
}

}  // namespace beluga

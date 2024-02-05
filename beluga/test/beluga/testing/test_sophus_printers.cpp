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

#include <sstream>

#include <gtest/gtest.h>

#include <beluga/testing/sophus_printers.hpp>

#include "beluga/test/utils/eigen_initializers.hpp"

namespace {

TEST(SophusPrinters, Matrix1i) {
  std::stringstream out;
  out << Eigen::Matrix<int, 1, 1>{42};
  ASSERT_EQ(out.str(), "(42)");
}

TEST(SophusPrinters, Vector3d) {
  std::stringstream out;
  out << Eigen::Vector3d{1.5, 2.5, 3.5};
  ASSERT_EQ(out.str(), "(1.5, 2.5, 3.5)");
}

TEST(SophusPrinters, Matrix2d) {
  std::stringstream out;
  out << testing::as<Eigen::Matrix2d>({{1.0, 2.0}, {3.0, 4.0}});
  ASSERT_EQ(out.str(), "(1, 2, 3, 4)");
}

TEST(SophusPrinters, Matrix3i) {
  std::stringstream out;
  out << testing::as<Eigen::Matrix3i>({{9, 8, 7}, {6, 5, 4}, {3, 2, 1}});
  ASSERT_EQ(out.str(), "(9, 8, 7, 6, 5, 4, 3, 2, 1)");
}

TEST(SophusPrinters, SO2dOneZero) {
  std::stringstream out;
  out << Sophus::SO2d{0.0};
  ASSERT_EQ(out.str(), "(1, 0)");
}

TEST(SophusPrinters, SO2dZeroOne) {
  std::stringstream out;
  out << Sophus::SO2d{0.0, 1.0};
  ASSERT_EQ(out.str(), "(0, 1)");
}

TEST(SophusPrinters, SE2dTwoThree) {
  std::stringstream out;
  out << Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{2.0, 3.0}};
  ASSERT_EQ(out.str(), "((1, 0), (2, 3))");
}

TEST(SophusPrinters, SE2dThreeTwo) {
  std::stringstream out;
  out << Sophus::SE2d{Sophus::SO2d{0.0, 1.0}, Sophus::Vector2d{3.0, 2.0}};
  ASSERT_EQ(out.str(), "((0, 1), (3, 2))");
}

}  // namespace

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

#include <gtest/gtest.h>

#include <sstream>

#include <beluga_amcl/amcl_node_utils.hpp>

namespace {

TEST(MakeEigenCommaFormat, Vector) {
  auto eigen_format = beluga_amcl::utils::make_eigen_comma_format();
  std::stringstream out;
  out << Eigen::Vector3d{1.0, 2.0, 3.0}.format(eigen_format);
  ASSERT_EQ(out.str(), "1, 2, 3");
}

TEST(MakeEigenCommaFormat, Matrix) {
  auto eigen_format = beluga_amcl::utils::make_eigen_comma_format();
  std::stringstream out;
  Eigen::Matrix2d matrix;
  matrix << 1.0, 2.0, 3.0, 4.0;
  out << matrix.format(eigen_format);
  ASSERT_EQ(out.str(), "1, 2, 3, 4");
}

}  // namespace

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

#ifndef BELUGA_TEST_UTILS_EIGEN_INITIALIZERS_HPP
#define BELUGA_TEST_UTILS_EIGEN_INITIALIZERS_HPP

#include <initializer_list>

#include <range/v3/view/enumerate.hpp>

namespace testing {

template <typename Matrix, typename Scalar = typename Matrix::Scalar>
Matrix as(std::initializer_list<std::initializer_list<Scalar>> rows) {
  Matrix matrix;
  for (auto&& [i, row] : ranges::views::enumerate(rows)) {
    for (auto&& [j, coeff] : ranges::views::enumerate(row)) {
      matrix(static_cast<typename Matrix::StorageIndex>(i), static_cast<typename Matrix::StorageIndex>(j)) = coeff;
    }
  }
  return matrix;
}

}  // namespace testing

#endif

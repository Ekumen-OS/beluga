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

#ifndef BELUGA_EIGEN_COMPATIBILITY_HPP
#define BELUGA_EIGEN_COMPATIBILITY_HPP

#include <Eigen/Dense>

#if (EIGEN_WORLD_VERSION < 3) || (EIGEN_WORLD_VERSION == 3 && EIGEN_MAJOR_VERSION < 4)
namespace Eigen {  // NOLINT(readability-identifier-naming)
/// Type alias for single-column matrices, available starting Eigen 3.4
template <typename Scalar, int Dims>
using Vector = Eigen::Matrix<Scalar, Dims, 1>;
template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
template <typename Scalar>
using Matrix3X = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;
}  // namespace Eigen
#endif

#endif  // BELUGA_EIGEN_COMPATIBILITY_HPP

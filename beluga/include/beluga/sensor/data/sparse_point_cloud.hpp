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

#ifndef BELUGA_SENSOR_DATA_SPARSE_POINT_CLOUD_HPP
#define BELUGA_SENSOR_DATA_SPARSE_POINT_CLOUD_HPP

#include <cmath>

#include <ciabatta/ciabatta.hpp>

#include <sophus/types.hpp>

/**
 * \file
 * \brief Implementation of a sparse point cloud interface without memory-aligned data.
 *
 * \details
 * Assumes data is aligned as X, Y, Z, other datafields .
 * Only X, Y & Z must be the same type (float or double).
 * Other datafields can be different types.
 */

namespace beluga {

/**
 * \page SparsePointCloudPage Beluga named requirements: PointCloud
 *
 * A type `P` satisfies `PointCloud` requirements if given `p` a possibly
 * const instance of `P`:
 * - `P::Scalar` is defined and is a scalar type to be used for x, y and z coordinates values,
 * -  p.points()` returns a view to the unorganized 3D point collection of `Eigen::Map<Eigen::Vector3>` type,
 * - `p.origin()` returns the transform, of `Sophus::SE3d` type, from the global to local
 *   frame in the sensor space;
 */

/// Sparse Point Cloud 3D base type.
/**
 * When instantiated, it satisfies \ref PointCloudPage.
 *
 * \tparam Derived Concrete point cloud type. It must define
 * `Derived::ranges()`, `Derived::angles()`,
 * as described in \ref PointCloudPage.
 */
template <typename Derived>
class BaseSparsePointCloud : public ciabatta::ciabatta_top<Derived> {};

}  // namespace beluga

#endif

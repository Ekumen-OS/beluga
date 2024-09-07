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

#ifndef BELUGA_SENSOR_DATA_POINT_CLOUD_HPP
#define BELUGA_SENSOR_DATA_POINT_CLOUD_HPP

#include <cmath>

#include <ciabatta/ciabatta.hpp>

#include <sophus/types.hpp>


/**
 * \file
 * \brief Implementation of a point cloud interface.
 */

namespace beluga {

/**
 * \page PointCloudPage Beluga named requirements: PointCloud
 *
 * A type `L` satisfies `PointCloud` requirements if given `l` a possibly
 * const instance of `L`:
 * - `L::Scalar` is defined and is a scalar type to be used for x, y and z coordinates values,
 * -  l.points()` returns a view of a point cloud,
 * - `l.origin()` returns the transform, of `Sophus::SE3d` type, from the global to local
 *   frame in the sensor space;
 */

/// Point Cloud 3D base type.
/**
 * When instantiated, it satisfies \ref PointCloudPage.
 *
 * \tparam Derived Concrete point cloud type. It must define
 * `Derived::ranges()`, `Derived::angles()`,
 * as described in \ref PointCloudPage.
 */
template <typename Derived>
class BasePointCloud : public ciabatta::ciabatta_top<Derived> {};

}  // namespace beluga

#endif

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

#ifndef BELUGA_3D_EMBEDDING_HPP
#define BELUGA_3D_EMBEDDING_HPP

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

namespace beluga {

/// Transforms a SE3 transform into a SE2 transform, by flattening the Z axis.
inline Sophus::SE2d To2d(const Sophus::SE3d& tf) {
  return Sophus::SE2d{tf.angleZ(), tf.translation().head<2>()};
}

/// Embed a SE2 transform into 3D space with zero Z translation and only rotation about the Z axis.
inline Sophus::SE3d To3d(const Sophus::SE2d& tf) {
  return Sophus::SE3d{
      Sophus::SO3d::rotZ(tf.so2().log()), Eigen::Vector3d{tf.translation().x(), tf.translation().y(), 0.0}};
}

}  // namespace beluga

#endif

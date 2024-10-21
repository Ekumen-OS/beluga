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

#ifndef BELUGA_TEST_STATIC_SIMPLE_POINTCLOUD_INTERFACE_HPP
#define BELUGA_TEST_STATIC_SIMPLE_POINTCLOUD_INTERFACE_HPP

#include <range/v3/view/iota.hpp>
#include <sophus/se3.hpp>

namespace beluga::testing {

template <typename T>
class SparsePointCloud3 {
 public:
  explicit SparsePointCloud3(std::vector<Eigen::Vector3<T>> points, Sophus::SE3d origin = Sophus::SE3d{})
      : points_(std::move(points)), origin_(std::move(origin)) {}

  /// Get the point cloud frame origin in the filter frame.
  [[nodiscard]] const auto& origin() const { return origin_; }

  /// Get the unorganized 3D point collection as an Eigen Map<Eigen::Vector3>.
  [[nodiscard]] auto points() const {
    return ranges::views::iota(0, points_.size()) |
           ranges::views::transform([this](int i) { return Eigen::Map<const Eigen::Vector3<T>>(points_.at(i)); });
  }

 private:
  std::vector<Eigen::Vector3<T>> points_;
  Sophus::SE3d origin_;
};

}  // namespace beluga::testing

#endif

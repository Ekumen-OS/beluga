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

#ifndef BELUGA_ROS_SPARSE_POINT_CLOUD_HPP
#define BELUGA_ROS_SPARSE_POINT_CLOUD_HPP

#include <range/v3/view/iota.hpp>

#include <beluga/sensor/data/sparse_point_cloud.hpp>
#include <beluga/views/take_evenly.hpp>
#include <beluga_ros/messages.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Dense>

#include "beluga/eigen_compatibility.hpp"

/**
 * \file
 * \brief Implementation of `sensor_msgs/PointCloud2` wrapper type for messages without alignment constraints on the
 * stride.
 *
 * \details
 * Since alignment is not enforced, this allows for more flexible messages layouts, though it may lead to less efficient
 * access patterns in certain cases.
 */

namespace beluga_ros {

/// Thin wrapper type for 3D `sensor_msgs/PointCloud2` messages.
/// Assumes an XYZ... type message.
/// XYZ datafields must be the same type (float or double).
/// Other datafields can be different types.
template <typename T>
class SparsePointCloud3 : public beluga::BaseSparsePointCloud<SparsePointCloud3<T>> {
 public:
  /// PointCloud type
  using Scalar = T;

  /// Check type is float or double
  static_assert(
      std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
      "PointcloudSparse3 only supports float or double datatype");

  /// Constructor.
  ///
  /// \param cloud Point cloud message.
  /// \param origin Point cloud frame origin in the filter frame.
  explicit SparsePointCloud3(beluga_ros::msg::PointCloud2ConstSharedPtr cloud, Sophus::SE3d origin = Sophus::SE3d())
      : cloud_(std::move(cloud)), origin_(std::move(origin)) {
    assert(cloud_ != nullptr);
    constexpr uint8_t fieldType = sensor_msgs::typeAsPointFieldType<T>::value;
    // Check if point cloud is 3D
    if (cloud_->fields.size() < 3) {
      throw std::invalid_argument("PointCloud is not 3D");
    }
    // Check point cloud is XYZ... type
    if (cloud_->fields.at(0).name != "x" && cloud_->fields.at(1).name != "y" && cloud_->fields.at(2).name != "z") {
      throw std::invalid_argument("PointCloud not XYZ...");
    }
    // Check XYZ datatype is the same
    if (cloud_->fields.at(0).datatype != fieldType || cloud_->fields.at(1).datatype != fieldType ||
        cloud_->fields.at(2).datatype != fieldType) {
      throw std::invalid_argument("XYZ datatype are not same");
    }
  }

  /// Get the point cloud size.
  [[nodiscard]] std::size_t size() const {
    return static_cast<std::size_t>(cloud_->width) * cloud_->height;
  }

  /// Get the point cloud frame origin in the filter frame.
  [[nodiscard]] const auto& origin() const { return origin_; }

  /// Get the unorganized 3D point collection as an Eigen Map<Eigen::Vector3>.
  [[nodiscard]] auto points() const {
    beluga_ros::msg::PointCloud2ConstIterator<Scalar> iter_points(*cloud_, "x");
    return ranges::views::iota(0, static_cast<int>(cloud_->width * cloud_->height)) |
           ranges::views::transform([iter_points](int i) mutable {
             return Eigen::Map<const Eigen::Vector3<Scalar>>(&(iter_points + i)[0]);
           });
  }

 private:
  beluga_ros::msg::PointCloud2ConstSharedPtr cloud_;
  Sophus::SE3d origin_;
};

}  // namespace beluga_ros

#endif  // BELUGA_ROS_SPARSE_POINT_CLOUD_HPP

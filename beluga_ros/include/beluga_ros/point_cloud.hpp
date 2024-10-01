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

#ifndef BELUGA_ROS_POINT_CLOUD_HPP
#define BELUGA_ROS_POINT_CLOUD_HPP

#include <range/v3/view/iota.hpp>

#include <beluga/sensor/data/point_cloud.hpp>
#include <beluga/views/take_evenly.hpp>
#include <beluga_ros/messages.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Dense>

#include "beluga/eigen_compatibility.hpp"

/**
 * \file
 * \brief Implementation of `sensor_msgs/PointCloud2` wrapper type for messages with memory-aligned strides.
 *
 * \details
 * The stride calculation ensures that the memory layout of the point cloud data is aligned with the size of the data
 * type used for iteration (`iteratorType`). To maintain proper memory alignment, the stride must satisfy the condition:
 *
 * `cloud_->point_step % sizeof(iteratorType) == 0`
 *
 * This condition guarantees that `point_step` is a multiple of the size of the `iteratorType`, ensuring efficient
 * access to each point in the cloud.
 */

namespace beluga_ros {

/// Thin wrapper type for 3D `sensor_msgs/PointCloud2` messages.
template <typename T>
class PointCloud3 : public beluga::BasePointCloud<PointCloud3<T>> {
 public:
  /// PointCloud type
  using Scalar = T;

  /// Check type is float or double
  static_assert(
      std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
      "Pointcloud3 only supports float or double datatype");

  /// Constructor.
  ///
  /// \param cloud Point cloud message.
  /// \param origin Point cloud frame origin in the filter frame.
  explicit PointCloud3(beluga_ros::msg::PointCloud2ConstSharedPtr cloud, Sophus::SE3d origin = Sophus::SE3d())
      : cloud_(std::move(cloud)), origin_(std::move(origin)) {
    assert(cloud_ != nullptr);
    constexpr uint8_t fieldType = sensor_msgs::typeAsPointFieldType<T>::value;
    // Check if point cloud is 3D
    if (cloud_->fields.size() < 3) {
      throw std::invalid_argument("PointCloud is not 3D");
    }
    // Check point cloud is XYZ... type
    if (cloud_->fields.at(0).name != "x" || cloud_->fields.at(1).name != "y" || cloud_->fields.at(2).name != "z") {
      throw std::invalid_argument("PointCloud not XYZ...");
    }
    // Check XYZ datatype is the same
    if (cloud_->fields.at(0).datatype != fieldType || cloud_->fields.at(1).datatype != fieldType ||
        cloud_->fields.at(2).datatype != fieldType) {
      throw std::invalid_argument("XYZ datatype are not same");
    }
    // Check stride is divisible
    if (cloud_->point_step % sizeof(Scalar) != 0) {
      throw std::invalid_argument("Data is not memory-aligned");
    }
    stride_ = static_cast<int>(cloud_->point_step / sizeof(Scalar));
  }

  /// Get the point cloud frame origin in the filter frame.
  [[nodiscard]] const auto& origin() const { return origin_; }

  /// Get the unorganized 3D point collection as an Eigen Map<Eigen::Matrix3X>.
  [[nodiscard]] auto points() const {
    const beluga_ros::msg::PointCloud2ConstIterator<Scalar> iter_points(*cloud_, "x");
    return Eigen::Map<const Eigen::Matrix3X<Scalar>, 0, Eigen::OuterStride<>>(
        &iter_points[0], 3, cloud_->width * cloud_->height, stride_);
  }

 private:
  beluga_ros::msg::PointCloud2ConstSharedPtr cloud_;
  int stride_;
  Sophus::SE3d origin_;
};

}  // namespace beluga_ros

#endif  // BELUGA_ROS_POINT_CLOUD_HPP

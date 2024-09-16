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

/**
 * \file
 * \brief Implementation of `sensor_msgs/PointCloud2` wrapper type.
 */

namespace beluga_ros {

template <uint8_t T>
struct DataType;

template <>
struct DataType<beluga_ros::msg::PointFieldF32> {
  using iteratorType = float;
  using eigenType = const Eigen::Matrix3Xf;
};

template <>
struct DataType<beluga_ros::msg::PointFieldF64> {
  using iteratorType = double;
  using eigenType = const Eigen::Matrix3Xd;
};

/// Thin wrapper type for 3D `sensor_msgs/PointCloud2` messages.
/// Assumes an XYZ... type message.
/// Each field must have the same datatype.
/// The point cloud can't have invalid values, i.e., it must be dense.
template <uint8_t T = beluga_ros::msg::PointFieldF64>
class PointCloud3 : public beluga::BasePointCloud<PointCloud3<T>> {
 public:
  /// PointCloud data fields type
  using iteratorType = typename DataType<T>::iteratorType;
  using eigenType = typename DataType<T>::eigenType;

  /// Check type is float or double
  static_assert(
      std::is_same<iteratorType, float>::value || std::is_same<iteratorType, double>::value,
      "Pointcloud3 only supports float or double datatype");

  /// Constructor.
  ///
  /// \param cloud Point cloud message.
  /// \param origin Point cloud frame origin in the filter frame.
  explicit PointCloud3(beluga_ros::msg::PointCloud2ConstSharedPtr cloud, Sophus::SE3d origin = Sophus::SE3d())
      : cloud_(std::move(cloud)), origin_(std::move(origin)) {
    // Check there are not invalid values
    if (!cloud_->is_dense)
      throw std::invalid_argument("PointCloud is not dense");
    // Check if point cloud is 3D
    if (cloud_->fields.size() < 3)
      throw std::invalid_argument("PointCloud is not 3D");
    // Check point cloud is XYZ... type
    if (cloud_->fields.at(0).name != "x" && cloud_->fields.at(1).name != "y" && cloud_->fields.at(2).name != "z")
      throw std::invalid_argument("PointCloud not XYZ...");
    // Check all datatype is the same
    if (!std::all_of(
            cloud_->fields.begin(), cloud_->fields.end(), [&](const auto& field) { return field.datatype == T; })) {
      throw std::invalid_argument("Fields do not match pointcloud datatype");
    }
    assert(cloud_ != nullptr);
  }

  /// Get the point cloud frame origin in the filter frame.
  [[nodiscard]] const auto& origin() const { return origin_; }

  /// Get the unorganized 3D point collection as an Eigen Map.
  [[nodiscard]] auto points() const {
    beluga_ros::msg::PointCloud2ConstIterator<iteratorType> iterPoints(*cloud_, "x");
    int stride_step = static_cast<int>((cloud_->point_step + sizeof(iteratorType) - 1) / sizeof(iteratorType));
    Eigen::Map<eigenType, 0, Eigen::OuterStride<>> map(
        &iterPoints[0], 3, cloud_->width * cloud_->height, Eigen::OuterStride<>(stride_step));
    return map;
  }

 private:
  beluga_ros::msg::PointCloud2ConstSharedPtr cloud_;
  Sophus::SE3d origin_;
};

}  // namespace beluga_ros

#endif  // BELUGA_ROS_POINT_CLOUD_HPP

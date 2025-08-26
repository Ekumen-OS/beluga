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

/// Thin wrapper type for 3D `sensor_msgs/PointCloud2` messages with dense layouts.
/**
 * The layout of the point cloud message must include exactly three (3) fields: x, y, and z,
 * all of the same floating point datatype, representing cartesian coordinates for each point.
 *
 * \tparam T Scalar type for point coordinates. Must be floating point.
 * \tparam Strict If true, xyz fields' datatypes must match the expected scalar type
 * or the wrapper will throw on construction. If false, the wrapper will cast as necessary.
 */
template <typename T, bool Strict = true>
class PointCloud3 : public beluga::BasePointCloud<PointCloud3<T>> {
 public:
  /// Expected PointCloud2 fields type
  using Scalar = T;

  // Assert fields' type is floating point
  static_assert(
      std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
      "Pointcloud3 only supports floating point types");

  /// Constructor.
  ///
  /// \param cloud Point cloud message.
  /// \param origin Point cloud frame origin in the filter frame.
  /// \throws std::invalid_argument if `cloud` does not meet expectations.
  explicit PointCloud3(beluga_ros::msg::PointCloud2ConstSharedPtr cloud, Sophus::SE3d origin = Sophus::SE3d())
      : cloud_(std::move(cloud)), origin_(std::move(origin)) {
    assert(cloud_ != nullptr);
    if (cloud_->fields.size() != 3) {
      throw std::invalid_argument("point cloud must have exactly 3 fields");
    }
    const auto& field_0 = cloud_->fields.at(0);
    const auto& field_1 = cloud_->fields.at(1);
    const auto& field_2 = cloud_->fields.at(2);
    if (field_0.name != "x" || field_1.name != "y" || field_2.name != "z") {
      throw std::invalid_argument("point cloud layout is not xyz");
    }
    if (field_0.datatype != field_1.datatype || field_1.datatype != field_2.datatype) {
      throw std::invalid_argument("point cloud xyz datatypes are not all the same");
    }
    if constexpr (Strict) {
      if (field_0.datatype != sensor_msgs::typeAsPointFieldType<Scalar>::value) {
        throw std::invalid_argument("xyz datatype does not match the expected type");
      }
    } else {
      if (field_0.datatype != sensor_msgs::typeAsPointFieldType<float>::value &&
          field_0.datatype != sensor_msgs::typeAsPointFieldType<double>::value) {
        throw std::invalid_argument("xyz datatype is not floating point");
      }
    }
    constexpr auto float_datatype = sensor_msgs::typeAsPointFieldType<float>::value;
    const auto size_of_datatype = field_0.datatype == float_datatype ? sizeof(float) : sizeof(double);
    if (cloud_->point_step % size_of_datatype != 0) {
      throw std::invalid_argument("point cloud is not dense");
    }
  }

  /// Get the point cloud frame origin in the filter frame.
  [[nodiscard]] const auto& origin() const { return origin_; }

  /// Get cartesian points in the point cloud as a matrix.
  [[nodiscard]] auto points() const {
    if constexpr (!Strict) {
      const auto datatype = cloud_->fields.at(0).datatype;
      if (datatype != sensor_msgs::typeAsPointFieldType<Scalar>::value) {
        if (datatype == sensor_msgs::typeAsPointFieldType<float>::value) {
          return Eigen::Matrix3X<Scalar>(points_matrix<float>(*cloud_).template cast<Scalar>());
        }
        assert(datatype == sensor_msgs::typeAsPointFieldType<double>::value);
        return Eigen::Matrix3X<Scalar>(points_matrix<double>(*cloud_).template cast<Scalar>());
      }
      return Eigen::Matrix3X<Scalar>(points_matrix<Scalar>(*cloud_));
    } else {
      return points_matrix<Scalar>(*cloud_);
    }
  }

 private:
  template <typename U>
  static auto points_matrix(const beluga_ros::msg::PointCloud2& cloud) {
    const auto stride = static_cast<int>(cloud.point_step / sizeof(U));
    const beluga_ros::msg::PointCloud2ConstIterator<U> iter_points(cloud, "x");
    return Eigen::Map<const Eigen::Matrix3X<U>, 0, Eigen::OuterStride<>>(
        &iter_points[0], 3, cloud.width * cloud.height, stride);
  }

  beluga_ros::msg::PointCloud2ConstSharedPtr cloud_;
  Sophus::SE3d origin_;
};

// Non-strict aliases.
using PointCloud3d = PointCloud3<double, false>;
using PointCloud3f = PointCloud3<float, false>;

}  // namespace beluga_ros

#endif  // BELUGA_ROS_POINT_CLOUD_HPP

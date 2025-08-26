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

#include <Eigen/Dense>
#include <beluga/sensor/data/sparse_point_cloud.hpp>
#include <beluga/views/take_evenly.hpp>
#include <beluga_ros/messages.hpp>
#include <range/v3/view/any_view.hpp>
#include <range/v3/view/iota.hpp>
#include <sophus/se3.hpp>
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

/// Thin wrapper for 3D `sensor_msgs/PointCloud2` messages with potentially sparse layouts.
/**
 * The layout of the point cloud message is only required to include at least three (3) fields: x, y, and z,
 * all of the same floating point datatype, representing cartesian coordinates for each point. These fields
 * must be at the beginning of each point.
 *
 * \tparam T Scalar type for point coordinates. Must be floating point.
 * \tparam Strict If true, xyz fields' datatypes must match the expected scalar type
 * or the wrapper will throw on construction. If false, the wrapper will cast as necessary.
 */
template <typename T, bool Strict = true>
class SparsePointCloud3 : public beluga::BaseSparsePointCloud<SparsePointCloud3<T>> {
 public:
  /// Expected PointCloud2 fields type
  using Scalar = T;

  // Assert fields' type is floating point
  static_assert(
      std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
      "SparsePointCloud3 only supports floating point types");

  /// Constructor.
  ///
  /// \param cloud Point cloud message.
  /// \param origin Point cloud frame origin in the filter frame.
  /// \throws std::invalid_argument if `cloud` does not meet expectations.
  explicit SparsePointCloud3(beluga_ros::msg::PointCloud2ConstSharedPtr cloud, Sophus::SE3d origin = Sophus::SE3d())
      : cloud_(std::move(cloud)), origin_(std::move(origin)) {
    assert(cloud_ != nullptr);
    if (cloud_->fields.size() < 3) {
      throw std::invalid_argument("point cloud must have at least 3 fields");
    }
    const auto& field_0 = cloud_->fields.at(0);
    const auto& field_1 = cloud_->fields.at(1);
    const auto& field_2 = cloud_->fields.at(2);
    if (field_0.name != "x" || field_1.name != "y" || field_2.name != "z") {
      throw std::invalid_argument("point cloud layout is not xyz...");
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
  }

  /// Get the point cloud size.
  [[nodiscard]] std::size_t size() const { return static_cast<std::size_t>(cloud_->width) * cloud_->height; }

  /// Get the point cloud frame origin in the filter frame.
  [[nodiscard]] const auto& origin() const { return origin_; }

  /// Get the range of cartesian points in the point cloud.
  [[nodiscard]] auto points() const {
    if constexpr (!Strict) {
      const auto datatype = cloud_->fields.at(0).datatype;
      if (datatype != sensor_msgs::typeAsPointFieldType<Scalar>::value) {
        if (datatype == sensor_msgs::typeAsPointFieldType<float>::value) {
          return ranges::any_view<Eigen::Vector3<Scalar>>(
              points_view<float>(*cloud_) |
              ranges::views::transform([](auto point) { return point.template cast<Scalar>(); }));
        } else if (datatype == sensor_msgs::typeAsPointFieldType<double>::value) {
          return ranges::any_view<Eigen::Vector3<Scalar>>(
              points_view<double>(*cloud_) |
              ranges::views::transform([](auto point) { return point.template cast<Scalar>(); }));
        } else {
          throw std::runtime_error("unexpected point cloud datatype");
        }
      }
      return ranges::any_view<Eigen::Vector3<Scalar>>(points_view<Scalar>(*cloud_));
    } else {
      return points_view<Scalar>(*cloud_);
    }
  }

 private:
  template <typename U>
  static auto points_view(const beluga_ros::msg::PointCloud2& cloud) {
    beluga_ros::msg::PointCloud2ConstIterator<U> iter_points(cloud, "x");
    return ranges::views::iota(0, static_cast<int>(cloud.width * cloud.height)) |
           ranges::views::transform(
               [iter_points](int i) mutable { return Eigen::Map<const Eigen::Vector3<U>>(&(iter_points + i)[0]); });
  }

  beluga_ros::msg::PointCloud2ConstSharedPtr cloud_;
  Sophus::SE3d origin_;
};

// Non-strict aliases.
using SparsePointCloud3d = SparsePointCloud3<double, false>;
using SparsePointCloud3f = SparsePointCloud3<float, false>;

}  // namespace beluga_ros

#endif  // BELUGA_ROS_SPARSE_POINT_CLOUD_HPP

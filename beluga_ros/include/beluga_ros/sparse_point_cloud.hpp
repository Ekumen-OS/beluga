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
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_field_conversion.hpp>

#include <beluga/sensor/data/sparse_point_cloud.hpp>
#include <beluga/views/take_evenly.hpp>
#include <beluga_ros/messages.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Dense>

#include "beluga/eigen_compatibility.hpp"
#include <cstring>
#include <stdexcept>

/**
 * \file
 * \brief Implementation of `sensor_msgs/PointCloud2` wrapper type for messages without alignment constraints on the
 * stride.
 *
 * \details
 * Since alignment is not enforced, this allows for more flexible messages layouts, though it may lead to less efficient
 * access patterns in certain cases.
 */

 namespace {
  inline size_t point_field_type_size(uint8_t datatype) {
      using sensor_msgs::msg::PointField;
      switch (datatype) {
          case PointField::INT8:
          case PointField::UINT8:
              return 1;
          case PointField::INT16:
          case PointField::UINT16:
              return 2;
          case PointField::INT32:
          case PointField::UINT32:
          case PointField::FLOAT32:
              return 4;
          case PointField::FLOAT64:
              return 8;
          default:
              throw std::runtime_error("Unknown PointField datatype");
      }
  }
}

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
    : origin_(std::move(origin)) {
    assert(cloud != nullptr);

    constexpr uint8_t fieldType = sensor_msgs::typeAsPointFieldType<T>::value;

    // Check if point cloud is 3D
    if (cloud->fields.size() < 3) {
      throw std::invalid_argument("PointCloud is not 3D");
    }

    // Check point cloud is XYZ... type
    if (cloud->fields.at(0).name != "x" || cloud->fields.at(1).name != "y" || cloud->fields.at(2).name != "z") {
      throw std::invalid_argument("PointCloud not XYZ...");
    }

    // If datatype already matches, store original
    if (cloud->fields.at(0).datatype == fieldType &&
        cloud->fields.at(1).datatype == fieldType &&
        cloud->fields.at(2).datatype == fieldType) {
      cloud_ = std::move(cloud);
      return;
    }

    // If FLOAT64 → FLOAT32 conversion is needed
    if constexpr (std::is_same_v<T, float>) {
      if (cloud->fields.at(0).datatype == sensor_msgs::msg::PointField::FLOAT64 &&
          cloud->fields.at(1).datatype == sensor_msgs::msg::PointField::FLOAT64 &&
          cloud->fields.at(2).datatype == sensor_msgs::msg::PointField::FLOAT64) {

        auto converted = std::make_shared<beluga_ros::msg::PointCloud2>();
        *converted = *cloud; // Copy metadata

        // Update datatype & point_step for XYZ
        for (int i = 0; i < 3; ++i) {
          converted->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
          converted->fields[i].count = 1;
        }

        converted->point_step = static_cast<uint32_t>(
          cloud->point_step - 3 * sizeof(double) + 3 * sizeof(float)
        );
        converted->data.resize(converted->width * converted->height * converted->point_step);

        // Copy with safe casting
        const uint8_t* src_ptr = cloud->data.data();
        uint8_t* dst_ptr = converted->data.data();

        for (size_t p = 0; p < cloud->width * cloud->height; ++p) {
          // Convert XYZ
          double dx, dy, dz;
          std::memcpy(&dx, src_ptr + cloud->fields[0].offset, sizeof(double));
          std::memcpy(&dy, src_ptr + cloud->fields[1].offset, sizeof(double));
          std::memcpy(&dz, src_ptr + cloud->fields[2].offset, sizeof(double));

          float fx = static_cast<float>(dx);
          float fy = static_cast<float>(dy);
          float fz = static_cast<float>(dz);

          std::memcpy(dst_ptr + converted->fields[0].offset, &fx, sizeof(float));
          std::memcpy(dst_ptr + converted->fields[1].offset, &fy, sizeof(float));
          std::memcpy(dst_ptr + converted->fields[2].offset, &fz, sizeof(float));

          // Copy other fields as-is
          for (const auto& f : cloud->fields) {
            if (f.name != "x" && f.name != "y" && f.name != "z") {
              std::memcpy(
                  dst_ptr + f.offset,
                  src_ptr + f.offset,
                  f.count * point_field_type_size(f.datatype)
                  //f.count * sensor_msgs::getPointCloud2PointFieldSize(f.datatype)
              );
            }
          }
          // // Convert XYZ double → float
          // double dx, dy, dz;
          // sensor_msgs::readPointCloud2BufferValue<double>(src_ptr + cloud->fields[0].offset, cloud->fields[0].datatype, dx);
          // sensor_msgs::readPointCloud2BufferValue<double>(src_ptr + cloud->fields[1].offset, cloud->fields[1].datatype, dy);
          // sensor_msgs::readPointCloud2BufferValue<double>(src_ptr + cloud->fields[2].offset, cloud->fields[2].datatype, dz);

          // float fx = static_cast<float>(dx);
          // float fy = static_cast<float>(dy);
          // float fz = static_cast<float>(dz);

          // sensor_msgs::writePointCloud2BufferValue<float>(dst_ptr + converted->fields[0].offset, converted->fields[0].datatype, fx);
          // sensor_msgs::writePointCloud2BufferValue<float>(dst_ptr + converted->fields[1].offset, converted->fields[1].datatype, fy);
          // sensor_msgs::writePointCloud2BufferValue<float>(dst_ptr + converted->fields[2].offset, converted->fields[2].datatype, fz);

          // // Copy other fields as-is
          // for (const auto& f : cloud->fields) {
          //   if (f.name != "x" && f.name != "y" && f.name != "z") {
          //     // Read as the original type, write unchanged
          //     if (f.datatype == sensor_msgs::msg::PointField::FLOAT64) {
          //       double val;
          //       sensor_msgs::readPointCloud2BufferValue<double>(src_ptr + f.offset, f.datatype, val);
          //       sensor_msgs::writePointCloud2BufferValue<double>(dst_ptr + f.offset, f.datatype, val);
          //     } else if (f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
          //       float val;
          //       sensor_msgs::msg::readPointCloud2BufferValue<float>(src_ptr + f.offset, f.datatype, val);
          //       sensor_msgs::msg::writePointCloud2BufferValue<float>(dst_ptr + f.offset, f.datatype, val);
          //     }
          //     // Add more types here if needed
          //   }
          // }

          src_ptr += cloud->point_step;
          dst_ptr += converted->point_step;
        }

        cloud_ = std::move(converted);
        return;
      }
    }

    throw std::invalid_argument("Unsupported XYZ datatype conversion");
  }

  /// Get the point cloud size.
  [[nodiscard]] std::size_t size() const { return static_cast<std::size_t>(cloud_->width) * cloud_->height; }

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

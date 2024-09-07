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

#ifndef BELUGA_ROS_POINT_CLOUD_HPP
#define BELUGA_ROS_POINT_CLOUD_HPP

#include <range/v3/view/iota.hpp>

#include <beluga/sensor/data/point_cloud.hpp>
#include <beluga/views/take_evenly.hpp>
#include <beluga_ros/messages.hpp>

#include <sophus/se3.hpp>

/**
 * \file
 * \brief Implementation of `sensor_msgs/PointCloud2` wrapper type.
 */

namespace beluga_ros {

/// Thin wrapper type for 3D `sensor_msgs/PointCloud2` messages.
class PointCloud2 : public beluga::BasePointCloud<PointCloud2> {
 public:
  /// Range type.
  using Scalar = double;

  /// Constructor.
  ///
  /// \param cloud Point cloud message.
  /// \param origin Point cloud frame origin in the filter frame.
  explicit PointCloud2(
      beluga_ros::msg::PointCloud2ConstSharedPtr cloud,
      Sophus::SE3d origin = Sophus::SE3d())
      : cloud_(std::move(cloud)),
        origin_(std::move(origin)),
        iter_points_(*cloud_, "x") {
    assert(cloud_ != nullptr);
  }

  /// Get the point cloud frame origin in the filter frame.
  [[nodiscard]] const auto& origin() const { return origin_; }

  /// Get point cloud view as a tuple.
  [[nodiscard]] auto points() const {
    return ranges::views::iota(0, static_cast<int>(cloud_->width * cloud_->height)) |
           ranges::views::transform([this](int i) {
             return std::make_tuple(static_cast<Scalar>(iter_points_[3 * i + 0]), 
                                    static_cast<Scalar>(iter_points_[3 * i + 1]),
                                    static_cast<Scalar>(iter_points_[3 * i + 2]));
           });
  }

 private:
  beluga_ros::msg::PointCloud2ConstSharedPtr cloud_;
  Sophus::SE3d origin_;
  beluga_ros::msg::PointCloud2ConstIterator<float> iter_points_;
};

}  // namespace beluga_ros

#endif  // BELUGA_ROS_POINT_CLOUD_HPP

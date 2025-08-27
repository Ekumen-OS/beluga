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

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <range/v3/range/conversion.hpp>

#include <Eigen/Dense>

#include "beluga/eigen_compatibility.hpp"
#include "beluga_ros/point_cloud.hpp"
#include "beluga_ros/test/point_cloud_utils.hpp"

namespace {

using beluga_ros::testing::make_ixyz_pointcloud;
using beluga_ros::testing::make_point_data;
using beluga_ros::testing::make_uninitialized_pointcloud;
using beluga_ros::testing::make_xy_pointcloud;
using beluga_ros::testing::make_xyz_int32_pointcloud;
using beluga_ros::testing::make_xyz_pointcloud;
using beluga_ros::testing::make_xyzi_pointcloud;
using beluga_ros::testing::make_zxyi_pointcloud;

constexpr unsigned int kUnorderedWidth = 5;
constexpr unsigned int kUnorderedHeight = 1;

TEST(TestPointCloud, FloatXYZPoints) {
  const auto point_data = make_point_data<float>(kUnorderedWidth * kUnorderedHeight);
  const auto message = make_xyz_pointcloud<float>(kUnorderedWidth, kUnorderedHeight, point_data);
  const auto cloud = beluga_ros::PointCloud3f(message);
  const auto matrix = cloud.points();
  for (unsigned i = 0; i < matrix.cols(); ++i) {
    ASSERT_EQ(point_data.at(i).x(), matrix(0, i));
    ASSERT_EQ(point_data.at(i).y(), matrix(1, i));
    ASSERT_EQ(point_data.at(i).z(), matrix(2, i));
  }
}

TEST(TestPointCloud, FloatXYZIPointsFails) {
  const auto point_data = make_point_data<float>(kUnorderedWidth * kUnorderedHeight);
  const auto message = make_xyzi_pointcloud<float>(kUnorderedWidth, kUnorderedHeight, point_data);
  ASSERT_THROW((beluga_ros::PointCloud3f(message)), std::invalid_argument);
}

TEST(TestPointCloud, EmptyXYZPoints) {
  const auto message = make_xyz_pointcloud<float>(0);
  const auto cloud = beluga_ros::PointCloud3f(message);
  const auto matrix = cloud.points();
  ASSERT_EQ(matrix.cols(), 0);
}

TEST(TestPointCloud, NoXYZIPoints) {
  const auto message = make_xyzi_pointcloud<float>(0);
  ASSERT_THROW((beluga_ros::PointCloud3f(message)), std::invalid_argument);
}

TEST(TestPointCloud, Non3DCloudFails) {
  const auto message = make_xy_pointcloud<float>(kUnorderedWidth * kUnorderedHeight);
  ASSERT_THROW((beluga_ros::PointCloud3f(message)), std::invalid_argument);
}

TEST(TestPointCloud, BadFieldsFail) {
  auto message = make_uninitialized_pointcloud();
  message->width = 4;
  message->height = 1;
  message->fields.clear();
  message->fields.reserve(4);
  message->is_dense = static_cast<decltype(message->is_dense)>(true);
  ASSERT_THROW((beluga_ros::PointCloud3f(message)), std::invalid_argument);
}

TEST(TestPointCloud, StrictFloatXYZFromDoubleFails) {
  constexpr bool kStrict = true;
  const auto message = make_xyz_pointcloud<double>(kUnorderedWidth * kUnorderedHeight);
  ASSERT_THROW((beluga_ros::PointCloud3<float, kStrict>(message)), std::invalid_argument);
}

TEST(TestPointCloud, StrictDoubleXYZFromFloatFails) {
  constexpr bool kStrict = true;
  const auto message = make_xyz_pointcloud<float>(kUnorderedWidth * kUnorderedHeight);
  ASSERT_THROW((beluga_ros::PointCloud3<double, kStrict>(message)), std::invalid_argument);
}

TEST(TestPointCloud, NonStrictFloatFromDouble) {
  const auto point_data = make_point_data<double>(kUnorderedWidth * kUnorderedHeight);
  const auto message = make_xyz_pointcloud<double>(kUnorderedWidth, kUnorderedHeight, point_data);
  const auto cloud = beluga_ros::PointCloud3f(message);
  const auto matrix = cloud.points();
  for (unsigned i = 0; i < matrix.cols(); ++i) {
    ASSERT_FLOAT_EQ(static_cast<float>(point_data.at(i).x()), matrix(0, i));
    ASSERT_FLOAT_EQ(static_cast<float>(point_data.at(i).y()), matrix(1, i));
    ASSERT_FLOAT_EQ(static_cast<float>(point_data.at(i).z()), matrix(2, i));
  }
}

TEST(TestPointCloud, NonStrictDoubleFromFloat) {
  const auto point_data = make_point_data<float>(kUnorderedWidth * kUnorderedHeight);
  const auto message = make_xyz_pointcloud<float>(kUnorderedWidth, kUnorderedHeight, point_data);
  const auto cloud = beluga_ros::PointCloud3d(message);
  const auto matrix = cloud.points();
  for (unsigned i = 0; i < matrix.cols(); ++i) {
    ASSERT_DOUBLE_EQ(static_cast<double>(point_data.at(i).x()), matrix(0, i));
    ASSERT_DOUBLE_EQ(static_cast<double>(point_data.at(i).y()), matrix(1, i));
    ASSERT_DOUBLE_EQ(static_cast<double>(point_data.at(i).z()), matrix(2, i));
  }
}

TEST(TestPointCloud, NonFloatPointsFail) {
  const auto message = make_xyz_int32_pointcloud(1);
  ASSERT_THROW((beluga_ros::PointCloud3f(message)), std::invalid_argument);
}

TEST(TestPointCloud, UninitializedFails) {
  auto message = make_uninitialized_pointcloud();
  ASSERT_THROW((beluga_ros::PointCloud3f(message)), std::invalid_argument);
}

TEST(TestPointCloud, BadIXYZLayoutFails) {
  const auto point_data = make_point_data<float>(kUnorderedWidth);
  const auto message = make_ixyz_pointcloud(point_data);
  ASSERT_THROW((beluga_ros::PointCloud3f(message)), std::invalid_argument);
}

TEST(TestPointCloud, BadZXYILayoutFails) {
  const auto point_data = make_point_data<float>(kUnorderedWidth);
  const auto message = make_zxyi_pointcloud(point_data);
  ASSERT_THROW((beluga_ros::PointCloud3f(message)), std::invalid_argument);
}

}  // namespace

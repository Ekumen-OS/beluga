// Copyright 2025 Ekumen, Inc.
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
#include "beluga_ros/messages.hpp"
#include "beluga_ros/sparse_point_cloud.hpp"
#include "beluga_ros/test/point_cloud_utils.hpp"

namespace {

using beluga_ros::testing::make_ixyz_pointcloud;
using beluga_ros::testing::make_ouster_pointcloud;
using beluga_ros::testing::make_point_data;
using beluga_ros::testing::make_robosense_pointcloud;
using beluga_ros::testing::make_uninitialized_pointcloud;
using beluga_ros::testing::make_velodyne_pointcloud;
using beluga_ros::testing::make_xy_pointcloud;
using beluga_ros::testing::make_xyz_different_types_pointcloud;
using beluga_ros::testing::make_xyz_int32_pointcloud;
using beluga_ros::testing::make_xyz_pointcloud;
using beluga_ros::testing::make_xyzi_pointcloud;
using beluga_ros::testing::make_zxyi_pointcloud;

constexpr unsigned int kUnorderedWidth = 5;
constexpr unsigned int kUnorderedHeight = 1;

TEST(TestSparsePointCloud, FloatXYZPoints) {
  const auto point_data = make_point_data<float>(kUnorderedWidth * kUnorderedHeight);
  const auto message = make_xyz_pointcloud<float>(kUnorderedWidth, kUnorderedHeight, point_data);

  const auto cloud_sparse = beluga_ros::SparsePointCloud3f(message);
  auto points = cloud_sparse.points() | ranges::to<std::vector>;
  ASSERT_EQ(points.size(), point_data.size());
  for (size_t i = 0; i < points.size(); ++i) {
    ASSERT_EQ(point_data.at(i), points.at(i));
  }
}

TEST(TestSparsePointCloud, StrictFloatXYZPoints) {
  constexpr bool kStrict = true;
  const auto point_data = make_point_data<float>(kUnorderedWidth * kUnorderedHeight);
  const auto message = make_xyz_pointcloud<float>(kUnorderedWidth, kUnorderedHeight, point_data);

  const auto cloud_sparse = beluga_ros::SparsePointCloud3<float, kStrict>(message);
  const auto points = cloud_sparse.points() | ranges::to<std::vector<Eigen::Vector3f>>;
  ASSERT_EQ(points.size(), point_data.size());
  for (size_t i = 0; i < points.size(); ++i) {
    ASSERT_EQ(point_data.at(i), points.at(i));
  }
}

TEST(TestSparsePointCloud, FloatXYZIPoints) {
  const auto point_data = make_point_data<float>(kUnorderedWidth * kUnorderedHeight);
  const auto message = make_xyzi_pointcloud<float>(kUnorderedWidth, kUnorderedHeight, point_data);

  const auto cloud_sparse = beluga_ros::SparsePointCloud3f(message);
  auto points = cloud_sparse.points() | ranges::to<std::vector>;
  ASSERT_EQ(points.size(), point_data.size());
  for (size_t i = 0; i < points.size(); ++i) {
    ASSERT_EQ(point_data.at(i), points.at(i));
  }
}

TEST(TestSparsePointCloud, DoubleXYZIPoints) {
  const auto point_data = make_point_data<double>(kUnorderedWidth * kUnorderedHeight);
  const auto message = make_xyzi_pointcloud<double>(kUnorderedWidth, kUnorderedHeight, point_data);

  const auto cloud_sparse = beluga_ros::SparsePointCloud3d(message);
  auto points = cloud_sparse.points() | ranges::to<std::vector>;
  ASSERT_EQ(points.size(), point_data.size());
  for (size_t i = 0; i < points.size(); ++i) {
    ASSERT_EQ(point_data.at(i), points.at(i));
  }
}

TEST(TestSparsePointCloud, EmptyXYZPoints) {
  const auto message = make_xyz_pointcloud<float>(0);

  const auto cloud_sparse = beluga_ros::SparsePointCloud3f(message);
  auto points = cloud_sparse.points() | ranges::to<std::vector>;
  ASSERT_EQ(points.size(), 0);
}

TEST(TestSparsePointCloud, EmptyXYZIPoints) {
  const auto message = make_xyzi_pointcloud<float>(0);

  const auto cloud_sparse = beluga_ros::SparsePointCloud3f(message);
  auto points = cloud_sparse.points() | ranges::to<std::vector>;
  ASSERT_EQ(points.size(), 0);
}

TEST(TestSparsePointCloud, Non3DCloudFails) {
  const auto message = make_xy_pointcloud<float>(kUnorderedWidth * kUnorderedHeight);
  ASSERT_THROW((beluga_ros::SparsePointCloud3f(message)), std::invalid_argument);
}

TEST(TestSparsePointCloud, BadFieldsFail) {
  auto message = make_uninitialized_pointcloud();
  message->width = 4;
  message->height = 1;
  message->fields.clear();
  message->fields.reserve(4);
  message->is_dense = static_cast<decltype(message->is_dense)>(true);
  ASSERT_THROW((beluga_ros::SparsePointCloud3f(message)), std::invalid_argument);
}

TEST(TestSparsePointCloud, DifferentXYZTypesFail) {
  const auto message = make_xyz_different_types_pointcloud(1);
  ASSERT_THROW((beluga_ros::SparsePointCloud3f(message)), std::invalid_argument);
}

TEST(TestSparsePointCloud, StrictFloatFromDoubleFails) {
  constexpr bool kStrict = true;
  const auto message = make_xyzi_pointcloud<double>(kUnorderedWidth * kUnorderedHeight);
  ASSERT_THROW((beluga_ros::SparsePointCloud3<float, kStrict>(message)), std::invalid_argument);
}

TEST(TestSparsePointCloud, StrictDoubleFromFloatFails) {
  constexpr bool kStrict = true;
  const auto message = make_xyzi_pointcloud<float>(kUnorderedWidth * kUnorderedHeight);
  ASSERT_THROW((beluga_ros::SparsePointCloud3<double, kStrict>(message)), std::invalid_argument);
}

TEST(TestSparsePointCloud, NonStrictFloatFromDouble) {
  const auto point_data = make_point_data<double>(kUnorderedWidth * kUnorderedHeight);
  const auto message = make_xyzi_pointcloud<double>(kUnorderedWidth, kUnorderedHeight, point_data);
  const auto cloud_sparse = beluga_ros::SparsePointCloud3f(message);
  auto points = cloud_sparse.points() | ranges::to<std::vector>;
  ASSERT_EQ(points.size(), point_data.size());
  for (size_t i = 0; i < points.size(); ++i) {
    ASSERT_FLOAT_EQ(static_cast<float>(point_data.at(i).x()), points.at(i).x());
    ASSERT_FLOAT_EQ(static_cast<float>(point_data.at(i).y()), points.at(i).y());
    ASSERT_FLOAT_EQ(static_cast<float>(point_data.at(i).z()), points.at(i).z());
  }
}

TEST(TestSparsePointCloud, NonStrictDoubleFromFloat) {
  const auto point_data = make_point_data<float>(kUnorderedWidth * kUnorderedHeight);
  const auto message = make_xyzi_pointcloud<float>(kUnorderedWidth, kUnorderedHeight, point_data);
  const auto cloud_sparse = beluga_ros::SparsePointCloud3d(message);
  auto points = cloud_sparse.points() | ranges::to<std::vector>;
  ASSERT_EQ(points.size(), point_data.size());
  for (size_t i = 0; i < points.size(); ++i) {
    ASSERT_DOUBLE_EQ(static_cast<double>(point_data.at(i).x()), points.at(i).x());
    ASSERT_DOUBLE_EQ(static_cast<double>(point_data.at(i).y()), points.at(i).y());
    ASSERT_DOUBLE_EQ(static_cast<double>(point_data.at(i).z()), points.at(i).z());
  }
}

TEST(TestSparsePointCloud, NonFloatPointsFail) {
  const auto message = make_xyz_int32_pointcloud(1);
  ASSERT_THROW((beluga_ros::SparsePointCloud3f(message)), std::invalid_argument);
}

TEST(TestSparsePointCloud, UninitializedFails) {
  auto message = make_uninitialized_pointcloud();
  ASSERT_THROW((beluga_ros::SparsePointCloud3f(message)), std::invalid_argument);
}

TEST(TestSparsePointCloud, BadIXYZLayoutFails) {
  const auto point_data = make_point_data<float>(kUnorderedWidth);
  const auto message = make_ixyz_pointcloud(point_data);
  ASSERT_THROW((beluga_ros::SparsePointCloud3f(message)), std::invalid_argument);
}

TEST(TestSparsePointCloud, BadZXYILayoutFails) {
  const auto point_data = make_point_data<float>(kUnorderedWidth);
  const auto message = make_zxyi_pointcloud(point_data);
  ASSERT_THROW((beluga_ros::SparsePointCloud3f(message)), std::invalid_argument);
}

TEST(TestSparsePointCloud, VelodyneLayout) {
  const auto point_data = make_point_data<float>(kUnorderedWidth);
  const auto message = make_velodyne_pointcloud(point_data);
  const auto cloud_sparse = beluga_ros::SparsePointCloud3f(message);
  auto points = cloud_sparse.points() | ranges::to<std::vector>;
  ASSERT_EQ(points.size(), point_data.size());
  for (size_t i = 0; i < points.size(); ++i) {
    ASSERT_EQ(point_data.at(i), points.at(i));
  }
}

TEST(TestSparsePointCloud, RobosenseLayout) {
  const auto point_data = make_point_data<float>(kUnorderedWidth);
  const auto message = make_robosense_pointcloud(point_data);
  const auto cloud_sparse = beluga_ros::SparsePointCloud3f(message);
  auto points = cloud_sparse.points() | ranges::to<std::vector>;
  ASSERT_EQ(points.size(), point_data.size());
  for (size_t i = 0; i < points.size(); ++i) {
    ASSERT_EQ(point_data.at(i), points.at(i));
  }
}

TEST(TestSparsePointCloud, OusterLayout) {
  const auto point_data = make_point_data<float>(kUnorderedWidth);
  const auto message = make_ouster_pointcloud(point_data);
  const auto cloud_sparse = beluga_ros::SparsePointCloud3f(message);
  auto points = cloud_sparse.points() | ranges::to<std::vector>;
  ASSERT_EQ(points.size(), point_data.size());
  for (size_t i = 0; i < points.size(); ++i) {
    ASSERT_EQ(point_data.at(i), points.at(i));
  }
}

}  // namespace

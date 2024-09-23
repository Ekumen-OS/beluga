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

#if BELUGA_ROS_VERSION == 1
#include <boost/smart_ptr.hpp>
#endif

#include <Eigen/Dense>

#include "beluga/eigen_compatibility.hpp"
#include "beluga_ros/messages.hpp"
#include "beluga_ros/point_cloud.hpp"
#include "beluga_ros/sparse_point_cloud.hpp"

namespace {

auto make_message() {
#if BELUGA_ROS_VERSION == 2
  return std::make_shared<beluga_ros::msg::PointCloud2>();
#elif BELUGA_ROS_VERSION == 1
  return boost::make_shared<beluga_ros::msg::PointCloud2>();
#endif
}

template <typename T, uint8_t U>
auto make_pointcloud(
    typename std::vector<T>::size_type& fields,
    const unsigned int width,
    const unsigned int height,
    const std::vector<Eigen::Vector3<T>>& point_data = {},
    const bool empty = false) {
  if (point_data.size() < static_cast<unsigned>(width * height) && !empty)
    throw std::invalid_argument("Not enough points");

  auto message = make_message();

  message->width = width;
  message->height = height;
  message->fields.clear();
  message->fields.reserve(fields);

  const std::vector<T> intensity_data = {static_cast<T>(1.1), static_cast<T>(2.2), static_cast<T>(3.3),
                                         static_cast<T>(4.4), static_cast<T>(5.5), static_cast<T>(6.6),
                                         static_cast<T>(7.7), static_cast<T>(8.8), static_cast<T>(9.9)};

  // XY pointclouds
  if (fields == 2) {
    // Set offset
    int offset = 0;
    offset = addPointField(*message, "x", 1, U, offset);
    offset = addPointField(*message, "y", 1, U, offset);

    // Set message params
    message->point_step = static_cast<unsigned int>(offset);
    message->row_step = message->width * message->point_step;
    message->is_dense = true;
    message->data.resize(message->point_step * message->width * message->height);

    // Return empty pointcloud
    if (empty)
      return message;

    // Create data iterators
    beluga_ros::msg::PointCloud2Iterator<T> iter_x(*message, "x");
    beluga_ros::msg::PointCloud2Iterator<T> iter_y(*message, "y");

    // Fill the PointCloud2 message
    for (const auto& point : point_data) {
      *iter_x = point.x();
      *iter_y = point.y();

      ++iter_x;
      ++iter_y;
    }
  }

  // XYZ pointclouds
  else if (fields == 3) {
    // Set offset
    int offset = 0;
    offset = addPointField(*message, "x", 1, U, offset);
    offset = addPointField(*message, "y", 1, U, offset);
    offset = addPointField(*message, "z", 1, U, offset);

    // Set message params
    message->point_step = static_cast<unsigned int>(offset);
    message->row_step = message->width * message->point_step;
    message->is_dense = true;
    message->data.resize(message->point_step * message->width * message->height);

    // Return empty pointcloud
    if (empty)
      return message;

    // Create data iterators
    beluga_ros::msg::PointCloud2Iterator<T> iter_x(*message, "x");
    beluga_ros::msg::PointCloud2Iterator<T> iter_y(*message, "y");
    beluga_ros::msg::PointCloud2Iterator<T> iter_z(*message, "z");

    // Fill the PointCloud2 message
    for (const auto& point : point_data) {
      *iter_x = point.x();
      *iter_y = point.y();
      *iter_z = point.z();

      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }

  // XYZI pointclouds
  else if (fields == 4) {
    // Set offset
    int offset = 0;
    offset = addPointField(*message, "x", 1, U, offset);
    offset = addPointField(*message, "y", 1, U, offset);
    offset = addPointField(*message, "z", 1, U, offset);
    offset = addPointField(*message, "intensity", 1, U, offset);

    // Set message params
    message->point_step = static_cast<unsigned int>(offset);
    message->row_step = message->width * message->point_step;
    message->is_dense = true;
    message->data.resize(message->point_step * message->width * message->height);

    // Return empty pointcloud
    if (empty)
      return message;

    // Create data iterators
    beluga_ros::msg::PointCloud2Iterator<T> iter_x(*message, "x");
    beluga_ros::msg::PointCloud2Iterator<T> iter_y(*message, "y");
    beluga_ros::msg::PointCloud2Iterator<T> iter_z(*message, "z");
    beluga_ros::msg::PointCloud2Iterator<T> iter_intensity(*message, "intensity");

    // Fill the PointCloud2 message
    typename std::vector<T>::size_type i = 0;
    for (const auto& point : point_data) {
      *iter_x = point.x();
      *iter_y = point.y();
      *iter_z = point.z();
      *iter_intensity = intensity_data.at(i);

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++i;
    }
  }

  // Error
  else {
    throw std::invalid_argument("Number of fields error");
  }
  return message;
}

TEST(TestPointCloud, XYZPointsUnorderedPC) {
  const auto origin = Sophus::SE3d{};
  // Define pointcloud params
  std::vector<float>::size_type fields = 3;
  unsigned int width = 1;
  unsigned int height = 5;
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F)
  };
  // clang-format on
  // Create point cloud message
  const auto message = make_pointcloud<float, beluga_ros::msg::PointField::FLOAT32>(fields, width, height, point_data);

  // Check aligned pointcloud
  const auto cloud = beluga_ros::PointCloud3<float>(message, origin);
  const auto map = cloud.points();
  // Check assert
  for (unsigned i = 0; i < map.cols(); ++i) {
    ASSERT_EQ(point_data.at(i).x(), map(0, i));
    ASSERT_EQ(point_data.at(i).y(), map(1, i));
    ASSERT_EQ(point_data.at(i).z(), map(2, i));
  }

  // Check sparse pointcloud
  const auto cloud_sparse = beluga_ros::SparsePointCloud3<float>(message, origin);
  auto vector = cloud_sparse.points();
  // Check assert
  for (size_t i = 0; i < vector.size(); ++i) {
    ASSERT_EQ(point_data.at(i), vector.at(i));
  }
}

TEST(TestPointCloud, XYZPointsOrderedPC) {
  const auto origin = Sophus::SE3d{};
  // Define pointcloud params
  std::vector<float>::size_type fields = 3;
  unsigned int width = 3;
  unsigned int height = 3;
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F),
    Eigen::Vector3f(16.0F, 17.0F, 18.0F),
    Eigen::Vector3f(19.0F, 20.0F, 21.0F),
    Eigen::Vector3f(22.0F, 23.0F, 24.0F),
    Eigen::Vector3f(25.0F, 26.0F, 27.0F)
  };
  // clang-format on
  // Create point cloud message
  const auto message = make_pointcloud<float, beluga_ros::msg::PointField::FLOAT32>(fields, width, height, point_data);

  // Check aligned pointcloud
  const auto cloud = beluga_ros::PointCloud3<float>(message, origin);
  const auto map = cloud.points();
  // Check assert
  for (unsigned i = 0; i < map.cols(); ++i) {
    ASSERT_EQ(point_data.at(i).x(), map(0, i));
    ASSERT_EQ(point_data.at(i).y(), map(1, i));
    ASSERT_EQ(point_data.at(i).z(), map(2, i));
  }

  // Check sparse pointcloud
  auto cloud_sparse = beluga_ros::SparsePointCloud3<float>(message, origin);
  auto vector = cloud_sparse.points();
  // Check assert
  for (size_t i = 0; i < vector.size(); ++i) {
    ASSERT_EQ(point_data.at(i), vector.at(i));
  }
}

TEST(TestPointCloud, XYZIPointsUnorderedPC) {
  const auto origin = Sophus::SE3d{};
  // Define pointcloud params
  std::vector<float>::size_type fields = 4;
  unsigned int width = 1;
  unsigned int height = 5;
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F)
  };
  // clang-format on
  // Create point cloud message
  const auto message = make_pointcloud<float, beluga_ros::msg::PointField::FLOAT32>(fields, width, height, point_data);

  // Check aligned pointcloud
  const auto cloud = beluga_ros::PointCloud3<float>(message, origin);
  const auto map = cloud.points();
  // Check assert
  for (unsigned i = 0; i < map.cols(); ++i) {
    ASSERT_EQ(point_data.at(i).x(), map(0, i));
    ASSERT_EQ(point_data.at(i).y(), map(1, i));
    ASSERT_EQ(point_data.at(i).z(), map(2, i));
  }

  // Check sparse pointcloud
  const auto cloud_sparse = beluga_ros::SparsePointCloud3<float>(message, origin);
  auto vector = cloud_sparse.points();
  // Check assert
  for (size_t i = 0; i < vector.size(); ++i) {
    ASSERT_EQ(point_data.at(i), vector.at(i));
  }
}

TEST(TestPointCloud, XYZIPointsOrderedPC) {
  const auto origin = Sophus::SE3d{};
  // Define pointcloud params
  std::vector<float>::size_type fields = 4;
  unsigned int width = 3;
  unsigned int height = 3;
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F),
    Eigen::Vector3f(16.0F, 17.0F, 18.0F),
    Eigen::Vector3f(19.0F, 20.0F, 21.0F),
    Eigen::Vector3f(22.0F, 23.0F, 24.0F),
    Eigen::Vector3f(25.0F, 26.0F, 27.0F)
  };
  // clang-format on
  // Create point cloud message
  const auto message = make_pointcloud<float, beluga_ros::msg::PointField::FLOAT32>(fields, width, height, point_data);

  // Check aligned pointcloud
  const auto cloud = beluga_ros::PointCloud3<float>(message, origin);
  const auto map = cloud.points();
  // Check assert
  for (unsigned i = 0; i < map.cols(); ++i) {
    ASSERT_EQ(point_data.at(i).x(), map(0, i));
    ASSERT_EQ(point_data.at(i).y(), map(1, i));
    ASSERT_EQ(point_data.at(i).z(), map(2, i));
  }

  // Check sparse pointcloud
  const auto cloud_sparse = beluga_ros::SparsePointCloud3<float>(message, origin);
  auto vector = cloud_sparse.points();
  // Check assert
  for (size_t i = 0; i < vector.size(); ++i) {
    ASSERT_EQ(point_data.at(i), vector.at(i));
  }
}

TEST(TestPointCloud, XYZIDoublePC) {
  const auto origin = Sophus::SE3d{};
  // Define pointcloud params
  std::vector<double>::size_type fields = 4;
  unsigned int width = 1;
  unsigned int height = 5;
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3d> point_data = {
    Eigen::Vector3d(1.0, 2.0, 3.0),
    Eigen::Vector3d(4.0, 5.0, 6.0),
    Eigen::Vector3d(7.0, 8.0, 9.0),
    Eigen::Vector3d(10.0, 11.0, 12.0),
    Eigen::Vector3d(13.0, 14.0, 15.0)
  };
  // clang-format on
  // Create point cloud message
  const auto message = make_pointcloud<double, beluga_ros::msg::PointField::FLOAT64>(fields, width, height, point_data);

  // Check aligned pointcloud
  const auto cloud = beluga_ros::PointCloud3<double>(message, origin);
  const auto map = cloud.points();
  // Check assert
  for (unsigned i = 0; i < map.cols(); ++i) {
    ASSERT_EQ(point_data.at(i).x(), map(0, i));
    ASSERT_EQ(point_data.at(i).y(), map(1, i));
    ASSERT_EQ(point_data.at(i).z(), map(2, i));
  }

  // Check sparse pointcloud
  const auto cloud_sparse = beluga_ros::SparsePointCloud3<double>(message, origin);
  auto vector = cloud_sparse.points();
  // Check assert
  for (size_t i = 0; i < vector.size(); ++i) {
    ASSERT_EQ(point_data.at(i), vector.at(i));
  }
}

TEST(TestPointCloud, XYZPointsEmptyUnorderedPC) {
  const auto origin = Sophus::SE3d{};
  // Define pointcloud params
  std::vector<float>::size_type fields = 3;
  unsigned int width = 1;
  unsigned int height = 5;
  const std::vector<Eigen::Vector3f>& point_data = {};
  bool empty = true;
  // Create point cloud message
  const auto message =
      make_pointcloud<float, beluga_ros::msg::PointField::FLOAT32>(fields, width, height, point_data, empty);

  // Check aligned pointcloud
  const auto cloud = beluga_ros::PointCloud3<float>(message, origin);
  const auto map = cloud.points();
  // Check assert
  for (unsigned i = 0; i < map.cols(); ++i) {
    ASSERT_EQ(message->data.at(3 * i + 0), map(0, i));
    ASSERT_EQ(message->data.at(3 * i + 1), map(1, i));
    ASSERT_EQ(message->data.at(3 * i + 2), map(2, i));
  }

  // Check sparse pointcloud
  const auto cloud_sparse = beluga_ros::SparsePointCloud3<float>(message, origin);
  auto vector = cloud_sparse.points();
  // Check assert
  for (size_t i = 0; i < vector.size(); ++i) {
    ASSERT_EQ(message->data.at(3 * i + 0), vector.at(i).x());
    ASSERT_EQ(message->data.at(3 * i + 1), vector.at(i).y());
    ASSERT_EQ(message->data.at(3 * i + 2), vector.at(i).z());
  }
}

TEST(TestPointCloud, XYZIPointsEmptyUnorderedPC) {
  const auto origin = Sophus::SE3d{};
  // Define pointcloud params
  std::vector<float>::size_type fields = 4;
  unsigned int width = 1;
  unsigned int height = 5;
  const std::vector<Eigen::Vector3f>& point_data = {};
  bool empty = true;
  // Create point cloud message
  const auto message =
      make_pointcloud<float, beluga_ros::msg::PointField::FLOAT32>(fields, width, height, point_data, empty);

  // Check aligned pointcloud
  const auto cloud = beluga_ros::PointCloud3<float>(message, origin);
  const auto map = cloud.points();
  // Check assert
  for (unsigned i = 0; i < map.cols(); ++i) {
    ASSERT_EQ(message->data.at(3 * i + 0), map(0, i));
    ASSERT_EQ(message->data.at(3 * i + 1), map(1, i));
    ASSERT_EQ(message->data.at(3 * i + 2), map(2, i));
  }

  // Check sparse pointcloud
  const auto cloud_sparse = beluga_ros::SparsePointCloud3<float>(message, origin);
  auto vector = cloud_sparse.points();
  // Check assert
  for (size_t i = 0; i < vector.size(); ++i) {
    ASSERT_EQ(message->data.at(3 * i + 0), vector.at(i).x());
    ASSERT_EQ(message->data.at(3 * i + 1), vector.at(i).y());
    ASSERT_EQ(message->data.at(3 * i + 2), vector.at(i).z());
  }
}

TEST(TestPointCloud, 2DUnorderedPC) {
  const auto origin = Sophus::SE3d{};
  // Define pointcloud params
  std::vector<float>::size_type fields = 2;
  unsigned int width = 1;
  unsigned int height = 5;
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F)
  };
  // clang-format on
  // Create point cloud message
  const auto message = make_pointcloud<float, beluga_ros::msg::PointField::FLOAT32>(fields, width, height, point_data);
  // Check assert aligned pointcloud
  ASSERT_THROW(beluga_ros::PointCloud3<float>(message, origin), std::invalid_argument);
  // Check assert sparse pointcloud
  ASSERT_THROW(beluga_ros::SparsePointCloud3<float>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, 2DOrderedPC) {
  const auto origin = Sophus::SE3d{};
  // Define pointcloud params
  std::vector<float>::size_type fields = 2;
  unsigned int width = 2;
  unsigned int height = 2;
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F)
  };
  // clang-format on
  // Create point cloud message
  const auto message = make_pointcloud<float, beluga_ros::msg::PointField::FLOAT32>(fields, width, height, point_data);
  // Check assert aligned pointcloud
  ASSERT_THROW(beluga_ros::PointCloud3<float>(message, origin), std::invalid_argument);
  // Check assert sparse pointcloud
  ASSERT_THROW(beluga_ros::SparsePointCloud3<float>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, EmptyFieldsPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 4;  // Number of points
  // Set the point fields to x, y, z and intensity
  std::vector<float>::size_type fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  message->is_dense = true;
  // Check assert aligned pointcloud
  ASSERT_THROW(beluga_ros::PointCloud3<float>(message, origin), std::invalid_argument);
  // Check assert sparse pointcloud
  ASSERT_THROW(beluga_ros::SparsePointCloud3<float>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, WrongTypePC) {
  const auto origin = Sophus::SE3d{};
  // Define pointcloud params
  std::vector<double>::size_type fields = 4;
  unsigned int width = 1;
  unsigned int height = 5;
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3d> point_data = {
    Eigen::Vector3d(1.0, 2.0, 3.0),
    Eigen::Vector3d(4.0, 5.0, 6.0),
    Eigen::Vector3d(7.0, 8.0, 9.0),
    Eigen::Vector3d(10.0, 11.0, 12.0),
    Eigen::Vector3d(13.0, 14.0, 15.0)
  };
  // clang-format on
  // Create point cloud message
  const auto message = make_pointcloud<double, beluga_ros::msg::PointField::FLOAT64>(fields, width, height, point_data);
  // Check assert aligned pointcloud
  ASSERT_THROW(beluga_ros::PointCloud3<float>(message, origin), std::invalid_argument);
  // Check assert sparse pointcloud
  ASSERT_THROW(beluga_ros::SparsePointCloud3<float>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, VoidPC) {
  const auto origin = Sophus::SE3d{};
  auto message = make_message();
  // Check assert aligned pointcloud
  ASSERT_THROW(beluga_ros::PointCloud3<float>(message, origin), std::invalid_argument);
  // Check assert sparse pointcloud
  ASSERT_THROW(beluga_ros::SparsePointCloud3<float>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, IXYZPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields to x, y, z and intensity
  std::vector<float>::size_type fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  // Set message params
  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F)
  };
  // clang-format on
  const std::vector<float> intensity_data = {1.1F, 2.2F, 3.3F, 4.4F, 5.5F};
  // Fill the PointCloud2 message
  for (size_t i = 0; i < point_data.size(); ++i) {
    *iter_x = point_data.at(i).x();
    *iter_y = point_data.at(i).y();
    *iter_z = point_data.at(i).z();
    *iter_intensity = intensity_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  // Check assert aligned pointcloud
  ASSERT_THROW(beluga_ros::PointCloud3<float>(message, origin), std::invalid_argument);
  // Check assert sparse pointcloud
  ASSERT_THROW(beluga_ros::SparsePointCloud3<float>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, ZXYIPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields to x, y, z and intensity
  std::vector<float>::size_type fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  // Set message params
  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F)
  };
  // clang-format on
  const std::vector<float> intensity_data = {1.1F, 2.2F, 3.3F, 4.4F, 5.5F};
  // Fill the PointCloud2 message
  for (size_t i = 0; i < point_data.size(); ++i) {
    *iter_x = point_data.at(i).x();
    *iter_y = point_data.at(i).y();
    *iter_z = point_data.at(i).z();
    *iter_intensity = intensity_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  // Check assert aligned pointcloud
  ASSERT_THROW(beluga_ros::PointCloud3<float>(message, origin), std::invalid_argument);
  // Check assert sparse pointcloud
  ASSERT_THROW(beluga_ros::SparsePointCloud3<float>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, Velodyne) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields as velodyne
  std::vector<float>::size_type fields = 6;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "ring", 1, beluga_ros::msg::PointField::UINT16, offset);
  offset = addPointField(*message, "time", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  // Set message params
  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  beluga_ros::msg::PointCloud2Iterator<std::uint16_t> iter_ring(*message, "ring");
  beluga_ros::msg::PointCloud2Iterator<float> iter_time(*message, "time");
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F)
  };
  // clang-format on
  const std::vector<float> intensity_data = {1.1F, 2.2F, 3.3F, 4.4F, 5.5F};
  const std::vector<std::uint16_t> ring_data = {1, 2, 3, 4, 5};
  const std::vector<float> time_data = {0.1F, 0.2F, 0.3F, 0.4F, 0.5F};
  // Fill the PointCloud2 message
  for (size_t i = 0; i < point_data.size(); ++i) {
    *iter_x = point_data.at(i).x();
    *iter_y = point_data.at(i).y();
    *iter_z = point_data.at(i).z();
    *iter_intensity = intensity_data.at(i);
    *iter_ring = ring_data.at(i);
    *iter_time = time_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
    ++iter_ring;
    ++iter_time;
  }
  // Check assert aligned pointcloud
  ASSERT_THROW(beluga_ros::PointCloud3<float>(message, origin), std::invalid_argument);

  // Check sparse pointcloud
  const auto cloud_sparse = beluga_ros::SparsePointCloud3<float>(message, origin);
  auto vector = cloud_sparse.points();
  // Check assert
  for (size_t i = 0; i < vector.size(); ++i) {
    ASSERT_EQ(point_data.at(i), vector.at(i));
  }
}

TEST(TestPointCloud, Robosense) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields as robosense
  std::vector<float>::size_type fields = 6;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "ring", 1, beluga_ros::msg::PointField::UINT16, offset);
  offset = addPointField(*message, "time", 1, beluga_ros::msg::PointField::FLOAT64, offset);
  // Set message params
  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  beluga_ros::msg::PointCloud2Iterator<std::uint16_t> iter_ring(*message, "ring");
  beluga_ros::msg::PointCloud2Iterator<double> iter_time(*message, "time");
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F)
  };
  // clang-format on
  const std::vector<float> intensity_data = {1.1F, 2.2F, 3.3F, 4.4F, 5.5F};
  const std::vector<std::uint16_t> ring_data = {1, 2, 3, 4, 5};
  const std::vector<double> time_data = {0.1, 0.2, 0.3, 0.4, 0.5};
  // Fill the PointCloud2 message
  for (size_t i = 0; i < point_data.size(); ++i) {
    *iter_x = point_data.at(i).x();
    *iter_y = point_data.at(i).y();
    *iter_z = point_data.at(i).z();
    *iter_intensity = intensity_data.at(i);
    *iter_ring = ring_data.at(i);
    *iter_time = time_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
    ++iter_ring;
    ++iter_time;
  }
  // Check assert aligned pointcloud
  ASSERT_THROW(beluga_ros::PointCloud3<float>(message, origin), std::invalid_argument);

  // Check sparse pointcloud
  const auto cloud_sparse = beluga_ros::SparsePointCloud3<float>(message, origin);
  auto vector = cloud_sparse.points();
  // Check assert
  for (size_t i = 0; i < vector.size(); ++i) {
    ASSERT_EQ(point_data.at(i), vector.at(i));
  }
}

TEST(TestPointCloud, Ouster) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields as ouster
  std::vector<float>::size_type fields = 9;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset += 4;
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "t", 1, beluga_ros::msg::PointField::UINT32, offset);
  offset = addPointField(*message, "reflectivity", 1, beluga_ros::msg::PointField::UINT16, offset);
  offset = addPointField(*message, "ring", 1, beluga_ros::msg::PointField::UINT8, offset);
  offset = addPointField(*message, "ambient", 1, beluga_ros::msg::PointField::UINT16, offset);
  offset = addPointField(*message, "range", 1, beluga_ros::msg::PointField::UINT32, offset);
  // Set message params
  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  beluga_ros::msg::PointCloud2Iterator<std::uint32_t> iter_time(*message, "t");
  beluga_ros::msg::PointCloud2Iterator<std::uint16_t> iter_reflectivity(*message, "reflectivity");
  beluga_ros::msg::PointCloud2Iterator<std::uint8_t> iter_ring(*message, "ring");
  beluga_ros::msg::PointCloud2Iterator<std::uint16_t> iter_ambient(*message, "ambient");
  beluga_ros::msg::PointCloud2Iterator<std::uint32_t> iter_range(*message, "range");
  // Create some raw data for the points
  // clang-format off
  const std::vector<Eigen::Vector3f> point_data = {
    Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F),
    Eigen::Vector3f(7.0F, 8.0F, 9.0F),
    Eigen::Vector3f(10.0F, 11.0F, 12.0F),
    Eigen::Vector3f(13.0F, 14.0F, 15.0F)
  };
  // clang-format on
  const std::vector<float> intensity_data = {1.1F, 2.2F, 3.3F, 4.4F, 5.5F};
  const std::vector<std::uint32_t> time_data = {1, 2, 3, 4, 5};
  const std::vector<std::uint16_t> reflectivity_data = {10, 9, 8, 7, 6};
  const std::vector<std::uint8_t> ring_data = {11, 12, 13, 14, 15};
  const std::vector<std::uint16_t> ambient_data = {20, 19, 18, 17, 16};
  const std::vector<std::uint32_t> range_data = {21, 22, 23, 24, 25};

  // Fill the PointCloud2 message
  for (size_t i = 0; i < point_data.size(); ++i) {
    *iter_x = point_data.at(i).x();
    *iter_y = point_data.at(i).y();
    *iter_z = point_data.at(i).z();
    *iter_intensity = intensity_data.at(i);
    *iter_time = time_data.at(i);
    *iter_reflectivity = reflectivity_data.at(i);
    *iter_ring = ring_data.at(i);
    *iter_ambient = ambient_data.at(i);
    *iter_range = range_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
    ++iter_time;
    ++iter_reflectivity;
    ++iter_ring;
    ++iter_ambient;
    ++iter_range;
  }
  // Check assert aligned pointcloud
  ASSERT_THROW(beluga_ros::PointCloud3<float>(message, origin), std::invalid_argument);

  // Check sparse pointcloud
  const auto cloud_sparse = beluga_ros::SparsePointCloud3<float>(message, origin);
  auto vector = cloud_sparse.points();
  // Check assert
  for (size_t i = 0; i < vector.size(); ++i) {
    ASSERT_EQ(point_data.at(i), vector.at(i));
  }
}

}  // namespace

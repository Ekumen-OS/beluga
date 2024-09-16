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

#include "beluga_ros/messages.hpp"
#include "beluga_ros/point_cloud.hpp"

namespace {

auto make_message() {
#if BELUGA_ROS_VERSION == 2
  return std::make_shared<beluga_ros::msg::PointCloud2>();
#elif BELUGA_ROS_VERSION == 1
  return boost::make_shared<beluga_ros::msg::PointCloud2>();
#endif
}

TEST(TestPointCloud, XYZPointsUnorderedPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields to x, y and z
  int fields = 3;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  // Create some raw data for the points
  const std::vector<float> point_data = {1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f, 8.0f,
                                         9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);

    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  auto cloud = beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin);
  auto map = cloud.points();
  // Check assert
  for (int i = 0; i < map.cols(); ++i) {
    for (int j = 0; j < map.rows(); ++j) {
      ASSERT_EQ(map(j, i), point_data.at(3 * i + j));
    }
  }
}

TEST(TestPointCloud, XYZPointsOrderedPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 3;   // Unordered point cloud
  message->height = 3;  // Number of points
  // Set the point fields to x, y and z
  int fields = 3;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  // Create some raw data for the points
  const std::vector<float> point_data = {1.0f,  2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,  8.0f,  9.0f,
                                         10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f, 18.0f,
                                         19.0f, 20.0f, 21.0f, 22.0f, 23.0f, 24.0f, 25.0f, 26.0f, 27.0f};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);

    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  auto cloud = beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin);
  auto map = cloud.points();
  // Check assert
  for (int i = 0; i < map.cols(); ++i) {
    for (int j = 0; j < map.rows(); ++j) {
      ASSERT_EQ(map(j, i), point_data.at(3 * i + j));
    }
  }
}

TEST(TestPointCloud, XYZIPointsUnorderedPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields to x, y, z and intensity
  int fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  // Create some raw data for the points
  const std::vector<float> point_data = {1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f, 8.0f,
                                         9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f};
  const std::vector<float> intensity_data = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);
    *iter_intensity = intensity_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  auto cloud = beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin);
  auto map = cloud.points();
  // Check assert
  for (int i = 0; i < map.cols(); ++i) {
    for (int j = 0; j < map.rows(); ++j) {
      ASSERT_EQ(map(j, i), point_data.at(3 * i + j));
    }
  }
}

TEST(TestPointCloud, XYZIPointsOrderedPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 3;  // Ordered point cloud
  message->height = 3;
  // Set the point fields to x, y, z and intensity
  int fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  // Create some raw data for the points
  const std::vector<float> point_data = {1.0f,  2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,  8.0f,  9.0f,
                                         10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f, 18.0f,
                                         19.0f, 20.0f, 21.0f, 22.0f, 23.0f, 24.0f, 25.0f, 26.0f, 27.0f};
  const std::vector<float> intensity_data = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f, 9.9f};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);
    *iter_intensity = intensity_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  auto cloud = beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin);
  auto map = cloud.points();
  // Check assert
  for (int i = 0; i < map.cols(); ++i) {
    for (int j = 0; j < map.rows(); ++j) {
      ASSERT_EQ(map(j, i), point_data.at(3 * i + j));
    }
  }
}

TEST(TestPointCloud, XYZIDoublePC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields to x, y, z and intensity
  int fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF64, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF64, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF64, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF64, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<double> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<double> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<double> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<double> iter_intensity(*message, "intensity");
  // Create some raw data for the points
  const std::vector<double> point_data = {1.0, 2.0,  3.0,  4.0,  5.0,  6.0,  7.0, 8.0,
                                          9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0};
  const std::vector<double> intensity_data = {1.1, 2.2, 3.3, 4.4, 5.5};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);
    *iter_intensity = intensity_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  auto cloud = beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF64>(message, origin);
  auto map = cloud.points();
  // Check assert
  for (int i = 0; i < map.cols(); ++i) {
    for (int j = 0; j < map.rows(); ++j) {
      ASSERT_EQ(map(j, i), point_data.at(3 * i + j));
    }
  }
}

TEST(TestPointCloud, XYZPointsEmptyUnorderedPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields to x, y and z
  int fields = 3;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);

  auto cloud = beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin);
  auto map = cloud.points();
  // Check assert
  for (int i = 0; i < map.cols(); ++i) {
    for (int j = 0; j < map.rows(); ++j) {
      ASSERT_EQ(map(j, i), message->data.at(3 * i + j));
    }
  }
}

TEST(TestPointCloud, XYZIPointsEmptyUnorderedPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 4;  // Number of points
  // Set the point fields to x, y, z and intensity
  int fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);

  auto cloud = beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin);
  auto map = cloud.points();
  // Check assert
  for (int i = 0; i < map.cols(); ++i) {
    for (int j = 0; j < map.rows(); ++j) {
      ASSERT_EQ(map(j, i), message->data.at(3 * i + j));
    }
  }
}

TEST(TestPointCloud, IXYZPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields to x, y, z and intensity
  int fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  // Create some raw data for the points
  const std::vector<float> point_data = {1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f, 8.0f,
                                         9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f};
  const std::vector<float> intensity_data = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);
    *iter_intensity = intensity_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  // Check assert
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, ZXYIPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields to x, y, z and intensity
  int fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  // Create some raw data for the points
  const std::vector<float> point_data = {1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f, 8.0f,
                                         9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f};
  const std::vector<float> intensity_data = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);
    *iter_intensity = intensity_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  /// Check assert
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, 2DUnorderedPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields to x and y
  int fields = 2;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  // Create some raw data for the points
  const std::vector<float> point_data = {1.0f, 2.0f, 4.0f, 5.0f, 7.0f, 8.0f, 10.0f, 11.0f, 12.0f, 13.0f};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 2; ++i) {
    *iter_x = point_data.at(2 * i + 0);
    *iter_y = point_data.at(2 * i + 1);

    ++iter_x;
    ++iter_y;
  }
  // Check assert
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, 2DOrderedPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 2;  // Ordered point cloud
  message->height = 2;
  // Set the point fields to x and y
  int fields = 2;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  // Create some raw data for the points
  const std::vector<float> point_data = {1.0f, 2.0f, 4.0f, 5.0f, 7.0f, 8.0f, 10.0f, 11.0f, 12.0f, 13.0f};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 2; ++i) {
    *iter_x = point_data.at(2 * i + 0);
    *iter_y = point_data.at(2 * i + 1);

    ++iter_x;
    ++iter_y;
  }
  // Check assert
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, EmptyFieldsPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 4;  // Number of points
  // Set the point fields to x, y, z and intensity
  int fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  message->is_dense = true;
  // Check assert
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, NotDensePC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 4;  // Number of points
  // Set the point fields to x, y, z and intensity
  int fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = false;
  message->data.resize(message->point_step * message->width * message->height);
  // Check assert
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, WrongTypePC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields to x, y, z and intensity
  int fields = 4;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF64, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF64, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF64, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF64, offset);
  // Set message params
  message->point_step = offset;
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  // Create data iterators
  beluga_ros::msg::PointCloud2Iterator<double> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<double> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<double> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<double> iter_intensity(*message, "intensity");
  // Create some raw data for the points
  const std::vector<double> point_data = {1.0, 2.0,  3.0,  4.0,  5.0,  6.0,  7.0, 8.0,
                                          9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0};
  const std::vector<double> intensity_data = {1.1, 2.2, 3.3, 4.4, 5.5};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);
    *iter_intensity = intensity_data.at(i);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  // Check assert
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, VoidPC) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  // Check assert
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, Velodyne) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields as velodyne
  int fields = 6;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "ring", 1, beluga_ros::msg::PointFieldU16, offset);
  offset = addPointField(*message, "time", 1, beluga_ros::msg::PointFieldF32, offset);
  // Set message params
  message->point_step = offset;
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
  const std::vector<float> point_data = {1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f, 8.0f,
                                         9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f};
  const std::vector<float> intensity_data = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f};
  const std::vector<std::uint16_t> ring_data = {1, 2, 3, 4, 5};
  const std::vector<float> time_data = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);
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
  // Check assert
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, Robosense) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields as robosense
  int fields = 6;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "ring", 1, beluga_ros::msg::PointFieldU16, offset);
  offset = addPointField(*message, "time", 1, beluga_ros::msg::PointFieldF64, offset);
  // Set message params
  message->point_step = offset;
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
  const std::vector<float> point_data = {1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f, 8.0f,
                                         9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f};
  const std::vector<float> intensity_data = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f};
  const std::vector<std::uint16_t> ring_data = {1, 2, 3, 4, 5};
  const std::vector<double> time_data = {0.1, 0.2, 0.3, 0.4, 0.5};
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);
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
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

TEST(TestPointCloud, Ouster) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1;   // Unordered point cloud
  message->height = 5;  // Number of points
  // Set the point fields as ouster
  int fields = 9;
  message->fields.clear();
  message->fields.reserve(fields);
  // Set offset
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointFieldF32, offset);
  offset += 4;
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointFieldF32, offset);
  offset = addPointField(*message, "t", 1, beluga_ros::msg::PointFieldU32, offset);
  offset = addPointField(*message, "reflectivity", 1, beluga_ros::msg::PointFieldU16, offset);
  offset = addPointField(*message, "ring", 1, beluga_ros::msg::PointFieldU8, offset);
  offset = addPointField(*message, "ambient", 1, beluga_ros::msg::PointFieldU16, offset);
  offset = addPointField(*message, "range", 1, beluga_ros::msg::PointFieldU32, offset);
  // Set message params
  message->point_step = offset;
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
  const std::vector<float> point_data = {1.0f, 2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f, 8.0f,
                                         9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f};
  const std::vector<float> intensity_data = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f};
  const std::vector<std::uint32_t> time_data = {1, 2, 3, 4, 5};
  const std::vector<std::uint16_t> reflectivity_data = {10, 9, 8, 7, 6};
  const std::vector<std::uint8_t> ring_data = {11, 12, 13, 14, 15};
  const std::vector<std::uint16_t> ambient_data = {20, 19, 18, 17, 16};
  const std::vector<std::uint32_t> range_data = {21, 22, 23, 24, 25};

  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size() / 3; ++i) {
    *iter_x = point_data.at(3 * i + 0);
    *iter_y = point_data.at(3 * i + 1);
    *iter_z = point_data.at(3 * i + 2);
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
  ASSERT_THROW(beluga_ros::PointCloud3<beluga_ros::msg::PointFieldF32>(message, origin), std::invalid_argument);
}

}  // namespace

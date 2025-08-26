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

#ifndef BELUGA_ROS_TEST_POINT_CLOUD_UTILS_HPP
#define BELUGA_ROS_TEST_POINT_CLOUD_UTILS_HPP

#include <memory>
#include <vector>
#include <optional>

#if BELUGA_ROS_VERSION == 1
#include <boost/smart_ptr.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#endif

#include <Eigen/Dense>

#include "beluga/eigen_compatibility.hpp"
#include "beluga_ros/messages.hpp"

namespace beluga_ros::testing {

auto make_uninitialized_pointcloud() {
#if BELUGA_ROS_VERSION == 2
  return std::make_shared<beluga_ros::msg::PointCloud2>();
#elif BELUGA_ROS_VERSION == 1
  return boost::make_shared<beluga_ros::msg::PointCloud2>();
#endif
}

template <typename T>
auto make_point_data(const size_t size) {
  std::vector<Eigen::Vector3<T>> points;
  points.reserve(size);
  for (size_t i = 0; i < size; ++i) {
    points.emplace_back(static_cast<T>(i), static_cast<T>(i * 2), static_cast<T>(i * 3));
  }
  return points;
}

template <typename T>
auto make_xy_pointcloud(
    const unsigned int width, std::optional<unsigned int> height = std::nullopt, 
    std::optional<std::vector<Eigen::Vector3<T>>> point_data = std::nullopt) {
  if (!height.has_value()) {
    height = width > 0 ? 1 : 0;
  }
  if (!point_data.has_value() && width * height.value() > 0) {
    point_data = make_point_data<T>(width * height.value());
  } else if (point_data.has_value() && point_data->size() != width * height.value()) {
    throw std::invalid_argument("point_data size does not match width * height");
  }

  auto message = make_uninitialized_pointcloud();

  message->width = width;
  message->height = height.value();
  message->fields.clear();
  message->fields.reserve(2);

  constexpr auto point_field_type = sensor_msgs::typeAsPointFieldType<T>::value;

  int offset = 0;
  offset = addPointField(*message, "x", 1, point_field_type, offset);
  offset = addPointField(*message, "y", 1, point_field_type, offset);

  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = static_cast<decltype(message->is_dense)>(true);
  message->data.resize(message->point_step * message->width * message->height);

  if (point_data.has_value()) {
    beluga_ros::msg::PointCloud2Iterator<T> iter_x(*message, "x");
    beluga_ros::msg::PointCloud2Iterator<T> iter_y(*message, "y");
    for (const auto& point : *point_data) {
      *iter_x = point.x();
      *iter_y = point.y();
      ++iter_x;
      ++iter_y;
    }
  }

  return message;
}

template <typename T>
auto make_xyz_pointcloud(
    const unsigned int width, std::optional<unsigned int> height = std::nullopt, 
    std::optional<std::vector<Eigen::Vector3<T>>> point_data = std::nullopt) {
  if (!height.has_value()) {
    height = width > 0 ? 1 : 0;
  }
  if (!point_data.has_value() && width * height.value() > 0) {
    point_data = make_point_data<T>(width * height.value());
  } else if (point_data.has_value() && point_data->size() != width * height.value()) {
    throw std::invalid_argument("point_data size does not match width * height");
  }

  auto message = make_uninitialized_pointcloud();

  message->width = width;
  message->height = height.value();
  message->fields.clear();
  message->fields.reserve(3);

  constexpr auto point_field_type = sensor_msgs::typeAsPointFieldType<T>::value;

  int offset = 0;
  offset = addPointField(*message, "x", 1, point_field_type, offset);
  offset = addPointField(*message, "y", 1, point_field_type, offset);
  offset = addPointField(*message, "z", 1, point_field_type, offset);

  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = static_cast<decltype(message->is_dense)>(true);
  message->data.resize(message->point_step * message->width * message->height);

  if (point_data.has_value()) {
    beluga_ros::msg::PointCloud2Iterator<T> iter_x(*message, "x");
    beluga_ros::msg::PointCloud2Iterator<T> iter_y(*message, "y");
    beluga_ros::msg::PointCloud2Iterator<T> iter_z(*message, "z");
    for (const auto& point : *point_data) {
      *iter_x = point.x();
      *iter_y = point.y();
      *iter_z = point.z();
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }

  return message;
}

template <typename T>
auto make_xyzi_pointcloud(
    const unsigned int width,
    std::optional<unsigned int> height = std::nullopt,
    std::optional<std::vector<Eigen::Vector3<T>>> point_data = std::nullopt,
    std::optional<std::vector<T>> intensity_data = std::nullopt) {
  if (!height.has_value()) {
    height = width > 0 ? 1 : 0;
  }
  if (!point_data.has_value() && width * height.value() > 0) {
    point_data = make_point_data<T>(width * height.value());
  }
  if (!intensity_data.has_value() && width * height.value() > 0) {
    intensity_data.emplace(width * height.value(), static_cast<T>(1.1));
  }

  auto message = make_uninitialized_pointcloud();

  message->width = width;
  message->height = height.value();
  message->fields.clear();
  message->fields.reserve(4);

  constexpr auto point_field_type = sensor_msgs::typeAsPointFieldType<T>::value;

  int offset = 0;
  offset = addPointField(*message, "x", 1, point_field_type, offset);
  offset = addPointField(*message, "y", 1, point_field_type, offset);
  offset = addPointField(*message, "z", 1, point_field_type, offset);
  offset = addPointField(*message, "intensity", 1, point_field_type, offset);

  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = static_cast<decltype(message->is_dense)>(true);
  message->data.resize(message->point_step * message->width * message->height);

  if (point_data.has_value()) {
    beluga_ros::msg::PointCloud2Iterator<T> iter_x(*message, "x");
    beluga_ros::msg::PointCloud2Iterator<T> iter_y(*message, "y");
    beluga_ros::msg::PointCloud2Iterator<T> iter_z(*message, "z");
    beluga_ros::msg::PointCloud2Iterator<T> iter_intensity(*message, "intensity");
    for (size_t i = 0; i < point_data->size(); ++i) {
      *iter_x = point_data->at(i).x();
      *iter_y = point_data->at(i).y();
      *iter_z = point_data->at(i).z();
      *iter_intensity = intensity_data->at(i);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
  }

  return message;
}

auto make_ixyz_pointcloud(const std::vector<Eigen::Vector3f>& point_data) {
  auto message = make_uninitialized_pointcloud();
  message->width = static_cast<unsigned int>(point_data.size());
  message->height = 1;
  message->fields.reserve(4);
  int offset = 0;
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  const std::vector<float> intensity_data(point_data.size(), 1.1F);
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
  return message;
}

auto make_zxyi_pointcloud(const std::vector<Eigen::Vector3f>& point_data) {
  auto message = make_uninitialized_pointcloud();
  message->width = static_cast<unsigned int>(point_data.size());
  message->height = 1;
  message->fields.reserve(4);
  int offset = 0;
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  const std::vector<float> intensity_data(point_data.size(), 1.1F);
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
  return message;
}

auto make_velodyne_pointcloud(const std::vector<Eigen::Vector3f>& point_data) {
  auto message = make_uninitialized_pointcloud();
  message->width = static_cast<unsigned int>(point_data.size());
  message->height = 1;
  message->fields.reserve(6);
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "ring", 1, beluga_ros::msg::PointField::UINT16, offset);
  offset = addPointField(*message, "time", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  beluga_ros::msg::PointCloud2Iterator<std::uint16_t> iter_ring(*message, "ring");
  beluga_ros::msg::PointCloud2Iterator<float> iter_time(*message, "time");
  const std::vector<float> intensity_data(point_data.size(), 1.1F);
  const std::vector<std::uint16_t> ring_data(point_data.size(), 1);
  const std::vector<float> time_data(point_data.size(), 0.1F);
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
  return message;
}

auto make_robosense_pointcloud(const std::vector<Eigen::Vector3f>& point_data) {
  auto message = make_uninitialized_pointcloud();
  message->width = static_cast<unsigned int>(point_data.size());
  message->height = 1;
  message->fields.reserve(6);
  int offset = 0;
  offset = addPointField(*message, "x", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "y", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "z", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "intensity", 1, beluga_ros::msg::PointField::FLOAT32, offset);
  offset = addPointField(*message, "ring", 1, beluga_ros::msg::PointField::UINT16, offset);
  offset = addPointField(*message, "time", 1, beluga_ros::msg::PointField::FLOAT64, offset);
  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  beluga_ros::msg::PointCloud2Iterator<std::uint16_t> iter_ring(*message, "ring");
  beluga_ros::msg::PointCloud2Iterator<double> iter_time(*message, "time");
  const std::vector<float> intensity_data(point_data.size(), 1.1F);
  const std::vector<std::uint16_t> ring_data(point_data.size(), 1);
  const std::vector<double> time_data(point_data.size(), 0.1);
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
  return message;
}

auto make_ouster_pointcloud(const std::vector<Eigen::Vector3f>& point_data) {
  auto message = make_uninitialized_pointcloud();
  message->width = static_cast<unsigned int>(point_data.size());
  message->height = 1;
  message->fields.reserve(9);
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
  message->point_step = static_cast<unsigned int>(offset);
  message->row_step = message->width * message->point_step;
  message->is_dense = true;
  message->data.resize(message->point_step * message->width * message->height);
  beluga_ros::msg::PointCloud2Iterator<float> iter_x(*message, "x");
  beluga_ros::msg::PointCloud2Iterator<float> iter_y(*message, "y");
  beluga_ros::msg::PointCloud2Iterator<float> iter_z(*message, "z");
  beluga_ros::msg::PointCloud2Iterator<float> iter_intensity(*message, "intensity");
  beluga_ros::msg::PointCloud2Iterator<std::uint32_t> iter_time(*message, "t");
  beluga_ros::msg::PointCloud2Iterator<std::uint16_t> iter_reflectivity(*message, "reflectivity");
  beluga_ros::msg::PointCloud2Iterator<std::uint8_t> iter_ring(*message, "ring");
  beluga_ros::msg::PointCloud2Iterator<std::uint16_t> iter_ambient(*message, "ambient");
  beluga_ros::msg::PointCloud2Iterator<std::uint32_t> iter_range(*message, "range");
  const std::vector<float> intensity_data(point_data.size(), 1.1F);
  const std::vector<std::uint32_t> time_data(point_data.size(), 1);
  const std::vector<std::uint16_t> reflectivity_data(point_data.size(), 10);
  const std::vector<std::uint8_t> ring_data(point_data.size(), 11);
  const std::vector<std::uint16_t> ambient_data(point_data.size(), 20);
  const std::vector<std::uint32_t> range_data(point_data.size(), 21);
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
  return message;
}

auto make_xyz_int32_pointcloud(const unsigned int size) {
  auto message = make_uninitialized_pointcloud();
  message->width = size;
  message->height = 1;
  message->is_dense = true;
  beluga_ros::msg::PointCloud2Modifier modifier(*message);
  modifier.setPointCloud2Fields(
      3, "x", 1, beluga_ros::msg::PointField::INT32, "y", 1, beluga_ros::msg::PointField::INT32, "z", 1,
      beluga_ros::msg::PointField::INT32);
  modifier.resize(size);
  return message;
}

}  // namespace beluga_ros::testing

#endif  // BELUGA_ROS_TEST_POINT_CLOUD_UTILS_HPP
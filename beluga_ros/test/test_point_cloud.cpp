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

#include "beluga_ros/point_cloud.hpp"
#include "beluga_ros/messages.hpp"

namespace {

auto make_message() {
#if BELUGA_ROS_VERSION == 2
  return std::make_shared<beluga_ros::msg::PointCloud2>();
#elif BELUGA_ROS_VERSION == 1
  return boost::make_shared<beluga_ros::msg::PointCloud2>();
#endif
}

TEST(TestPointCloud, PointsFromUnorderedPointCloudMessage) {
  auto message = make_message();
  const auto origin = Sophus::SE3d{};
  message->width = 1; // Unordered point cloud
  message->height = 4; // Number of points
  // Set the point fields to x, y, z, and rgb
  beluga_ros::msg::PointCloud2Modifier modifier(*message);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(message->width * message->height);
  // Create some raw data for the points
  std::vector<float> point_data = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
  // Initialize iterators for x, y, z and rgb fields
  beluga_ros::msg::PointCloud2Iterator<float> iter_points(*message, "x");
  // Fill the PointCloud2 message
  for (unsigned i = 0; i < point_data.size()/3; ++i)
  {
    iter_points[3 * i + 0] = point_data.at(3 * i + 0);
    iter_points[3 * i + 1] = point_data.at(3 * i + 1);
    iter_points[3 * i + 2] = point_data.at(3 * i + 2);
  }
  auto cloud = beluga_ros::PointCloud2(message, origin);
  int i = 0;
  for (const auto& point : cloud.points())
  {
    std::apply([&](auto x, auto y, auto z){
      ASSERT_EQ(x, point_data[3 * i + 0]);
      ASSERT_EQ(y, point_data[3 * i + 1]);
      ASSERT_EQ(z, point_data[3 * i + 2]);
    }, point);
    i++;
  }
  
}

}  // namespace

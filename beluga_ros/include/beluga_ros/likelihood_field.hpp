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

#ifndef BELUGA_ROS_LIKELIHOOD_FIELD_HPP
#define BELUGA_ROS_LIKELIHOOD_FIELD_HPP

#include <cstdint>

#include <beluga/sensor/data/value_grid.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

/**
 * \file
 * \brief Utilities for likelihood data over ROS interfaces.
 */

namespace beluga_ros {

inline void assign_likelihood_field(
    const beluga::ValueGrid2<float>& likelihood_field,
    nav_msgs::msg::OccupancyGrid& message) {
  // Set metadata
  message.info.width = static_cast<unsigned int>(likelihood_field.width());
  message.info.height = static_cast<unsigned int>(likelihood_field.height());
  message.info.resolution = static_cast<float>(likelihood_field.resolution());

  // Populate the data field with the grid data
  message.data.resize(likelihood_field.size());
  const auto& grid_data = likelihood_field.data();

  for (std::size_t i = 0; i < likelihood_field.size(); ++i) {
    // Convert T to int8_t if necessary
    message.data[i] = static_cast<int8_t>(grid_data[i]);
  }
}

}  // namespace beluga_ros

#endif  // BELUGA_ROS_LIKELIHOOD_FIELD_HPP

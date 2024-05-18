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

#ifndef BELUGA_ROS_PARTICLE_CLOUD_HPP
#define BELUGA_ROS_PARTICLE_CLOUD_HPP

#include <memory>
#include <type_traits>

#include <range/v3/range/primitives.hpp>
#include <range/v3/view/take_exactly.hpp>

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/types.hpp>

#include <beluga/primitives.hpp>
#include <beluga/views/sample.hpp>
#include <beluga_ros/messages.hpp>
#include <beluga_ros/tf2_sophus.hpp>

/**
 * \file
 * \brief Utilities for particle cloud I/O over ROS interfaces.
 */

namespace beluga_ros {

/// Assign a pose distribution to a particle cloud message.
/**
 * \param particles Pose distribution, as a particle cloud itself.
 * \param size Sample size of the particle cloud.
 * \param[out] message Particle cloud message to be assigned.
 * \tparam Particles A range type whose value type satisfies \ref ParticlePage "Particle" named requirements.
 */
template <
    class Particles,
    class Particle = ranges::range_value_t<Particles>,
    class State = typename beluga::state_t<Particle>,
    class Scalar = typename State::Scalar,
    typename = std::enable_if_t<
        std::is_same_v<State, typename Sophus::SE2<Scalar>> || std::is_same_v<State, typename Sophus::SE3<Scalar>>>>
beluga_ros::msg::PoseArray&
assign_particle_cloud(Particles&& particles, std::size_t size, beluga_ros::msg::PoseArray& message) {
  message.poses.clear();
  message.poses.reserve(size);
  for (const auto& particle : particles | beluga::views::sample | ranges::views::take_exactly(size)) {
    auto& pose = message.poses.emplace_back();
    tf2::toMsg(beluga::state(particle), pose);
  }
  return message;
}

/// Assign a distribution to a particle cloud message.
/**
 * Particle cloud sample size will match that of the given distribution.
 *
 * \param particles Distribution to sample, as a particle cloud itself.
 * \param message Particle cloud message to be assigned.
 * \return A particle cloud message with a sample size that matches that of the given distribution.
 * \tparam Particles A sized range type whose value type satisfies \ref ParticlePage "Particle" named requirements.
 */
template <class Particles, class Message>
Message& assign_particle_cloud(Particles&& particles, Message& message) {
  static_assert(ranges::sized_range<decltype(particles)>);
  return assign_particle_cloud(particles, ranges::size(particles), message);
}

}  // namespace beluga_ros

#endif  // BELUGA_ROS_PARTICLE_CLOUD_HPP

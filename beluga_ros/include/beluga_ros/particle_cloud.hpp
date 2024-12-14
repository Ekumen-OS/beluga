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

#include <cmath>
#include <map>
#include <memory>
#include <type_traits>

#include <range/v3/range/primitives.hpp>
#include <range/v3/view/take_exactly.hpp>

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/types.hpp>

#include <beluga/algorithm/spatial_hash.hpp>
#include <beluga/primitives.hpp>
#include <beluga/views/sample.hpp>
#include <beluga/views/zip.hpp>

#include <beluga_ros/messages.hpp>
#include <beluga_ros/tf2_eigen.hpp>
#include <beluga_ros/tf2_sophus.hpp>

/**
 * \file
 * \brief Utilities for particle cloud I/O over ROS interfaces.
 */

namespace beluga_ros {

namespace detail {

/// Make an RGBA color based on hue alone.
/**
 * Assumes both saturation and value to be 1.
 */
inline beluga_ros::msg::ColorRGBA alphaHueToRGBA(float hue, float alpha) {
  beluga_ros::msg::ColorRGBA message;
  // https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB_alternative
  // specialized for V = S = 1 and using single precision floats because
  // that is what the color message expects.
  const auto kr = std::fmod(5.0F + hue / 60.0F, 6.0F);
  const auto kg = std::fmod(3.0F + hue / 60.0F, 6.0F);
  const auto kb = std::fmod(1.0F + hue / 60.0F, 6.0F);
  message.r = 1.0F - 1.0F * std::max(0.0F, std::min({kr, 4.0F - kr, 1.0F}));
  message.g = 1.0F - 1.0F * std::max(0.0F, std::min({kg, 4.0F - kg, 1.0F}));
  message.b = 1.0F - 1.0F * std::max(0.0F, std::min({kb, 4.0F - kb, 1.0F}));
  message.a = alpha;
  return message;
}

/// std::equal_to equivalent for near equality operations.
template <class T>
struct almost_equal_to;

/// std::equal_to equivalent specialized for SE(2) types, with user resolutions.
template <typename Scalar>
struct almost_equal_to<Sophus::SE2<Scalar>> {
  /// Constructs near equality functor.
  /**
   * \param _linear_resolution Resolution for translational coordinates, in meters.
   * \param _angular_resolution Resolution for rotational coordinates, in radians.
   */
  explicit almost_equal_to(Scalar _linear_resolution, Scalar _angular_resolution)
      : linear_resolution(_linear_resolution), angular_resolution(_angular_resolution) {}

  /// Compares `a` and `b` for near equality.
  bool operator()(const Sophus::SE2<Scalar>& a, const Sophus::SE2<Scalar>& b) const {
    using std::abs;
    return (abs(a.translation().x() - b.translation().x()) < linear_resolution) &&
           (abs(a.translation().y() - b.translation().y()) < linear_resolution) &&
           (abs((a * b.inverse()).log().z()) < angular_resolution);
  }

  const Scalar linear_resolution;   ///< Resolution for translational coordinates, in meters.
  const Scalar angular_resolution;  ///< Resolution for rotational coordinates, in radians.
};

/// std::equal_to equivalent specialized for SE(3) types, with user resolutions.
template <typename Scalar>
struct almost_equal_to<Sophus::SE3<Scalar>> {
  /// Constructs near equality functor.
  /**
   * \param _linear_resolution Resolution for translational coordinates, in meters.
   * \param _angular_resolution Resolution for rotational coordinates, in radians.
   */
  explicit almost_equal_to(Scalar _linear_resolution, Scalar _angular_resolution)
      : linear_resolution(_linear_resolution), angular_resolution(_angular_resolution) {}

  /// Compares `a` and `b` for near equality.
  bool operator()(const Sophus::SE3<Scalar>& a, const Sophus::SE3<Scalar>& b) const {
    using std::abs;
    const Eigen::Vector<Scalar, 6> angles = (a * b.inverse()).log();
    return (abs(a.translation().x() - b.translation().x()) < linear_resolution) &&
           (abs(a.translation().y() - b.translation().y()) < linear_resolution) &&
           (abs(a.translation().z() - b.translation().z()) < linear_resolution) &&
           (abs(angles[3]) < angular_resolution) && (abs(angles[4]) < angular_resolution) &&
           (abs(angles[5]) < angular_resolution);
  }

  const Scalar linear_resolution;   ///< Resolution for translational coordinates, in meters.
  const Scalar angular_resolution;  ///< Resolution for rotational coordinates, in radians.
};

}  // namespace detail

/// Assign a pose distribution to a particle cloud message.
/**
 * \param particles Pose distribution, as a particle cloud itself.
 * \param size Sample size of the particle cloud.
 * \param[out] message Particle cloud message to be assigned.
 * \tparam Particles A sized range type whose value type satisfies \ref ParticlePage "Particle" named requirements.
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
  static_assert(ranges::sized_range<decltype(particles)>);
  message.poses.clear();
  if (ranges::size(particles) > 0) {
    message.poses.reserve(size);
    for (const auto& particle : particles | beluga::views::sample | ranges::views::take_exactly(size)) {
      auto& pose = message.poses.emplace_back();
      tf2::toMsg(beluga::state(particle), pose);
    }
  }
  return message;
}

/// Assign a distribution to a particle cloud message.
/**
 * Particle cloud sample size will match that of the given distribution.
 *
 * \param particles Distribution to sample, as a particle cloud itself.
 * \param[out] message Particle cloud message to be assigned.
 * \tparam Particles A sized range type whose value type satisfies \ref ParticlePage "Particle" named requirements.
 */
template <class Particles, class Message>
Message& assign_particle_cloud(Particles&& particles, Message& message) {
  static_assert(ranges::sized_range<decltype(particles)>);
  return assign_particle_cloud(std::forward<Particles>(particles), ranges::size(particles), message);
}

/// Assign a pose distribution to a markers message for visualization.
/**
 * \param particles Pose distribution, as a particle cloud itself.
 * \param linear_resolution Linear resolution, in meters, for visualization.
 * \param angular_resolution Angular resolution, in radians, for visualization.
 * \param[out] message Markers message to be assigned.
 * \tparam Particles A sized range type whose value type satisfies \ref ParticlePage "Particle" named requirements.
 */
template <
    class Particles,
    class Particle = ranges::range_value_t<Particles>,
    class State = typename beluga::state_t<Particle>,
    class Weight = typename beluga::weight_t<Particle>,
    class Scalar = typename State::Scalar,
    typename = std::enable_if_t<
        std::is_same_v<State, typename Sophus::SE2<Scalar>> || std::is_same_v<State, typename Sophus::SE3<Scalar>>>>
beluga_ros::msg::MarkerArray& assign_particle_cloud(
    Particles&& particles,
    Scalar linear_resolution,
    Scalar angular_resolution,
    beluga_ros::msg::MarkerArray& message) {
  // Particle weights from the filter may or may not be representative of the
  // true distribution. If we resampled, they are not, and there will be multiple copies
  // of the most likely candidates, all with unit weight. In this case the number of copies
  // is a proxy for the prob density at each candidate. If we did not resample before updating
  // the estimation and publishing this message (which can happen if the resample interval
  // is set to something other than 1), then all particles are expected to be different
  // and their weights are proportional to the prob density at each candidate.
  //
  // Only the combination of both the state distribution and the candidate weights together
  // provide information about the probability density at each candidate.
  auto max_bin_weight = Weight{1e-3};
  auto states = beluga::views::states(particles);
  auto weights = beluga::views::weights(particles);
  using StateHistogram = std::unordered_map<State, Weight, beluga::spatial_hash<State>, detail::almost_equal_to<State>>;
  auto histogram = StateHistogram{
      10U, beluga::spatial_hash<State>{linear_resolution, angular_resolution},
      detail::almost_equal_to<State>{linear_resolution, angular_resolution}};
  for (const auto& [state, weight] : beluga::views::zip(states, weights)) {
    auto& bin_weight = histogram[state];
    bin_weight += weight;
    if (bin_weight > max_bin_weight) {
      max_bin_weight = bin_weight;
    }
  }

  message.markers.clear();

  if (histogram.empty()) {
    return message;
  }

  // There are no arrow list markers yet (https://github.com/ros2/rviz/pull/972)
  // so we replicate them using a line list for arrow bodies and a triangle list
  // for arrow heads.
  constexpr auto kArrowBodyLength = Scalar{0.5};
  constexpr auto kArrowHeadLength = Scalar{0.1};
  constexpr auto kArrowHeadWidth = kArrowHeadLength / Scalar{5.0};
  constexpr auto kArrowLength = kArrowBodyLength + kArrowHeadLength;

  using Translation = typename State::TranslationType;
  const auto arrow_body_base = Translation::Zero();
  const auto arrow_head_tip = kArrowLength * Translation::UnitX();
  const auto arrow_head_base = kArrowBodyLength * Translation::UnitX();
  const auto arrow_head_right_corner =
      kArrowBodyLength * Translation::UnitX() - (kArrowHeadWidth / Scalar{2.0}) * Translation::UnitY();
  const auto arrow_head_left_corner =
      kArrowBodyLength * Translation::UnitX() + (kArrowHeadWidth / Scalar{2.0}) * Translation::UnitY();

  message.markers.reserve(2);
  auto& arrow_bodies = message.markers.emplace_back();
  arrow_bodies.id = 0;
  arrow_bodies.ns = "bodies";
  arrow_bodies.color.a = 1.0;
  arrow_bodies.pose.orientation.w = 1.0;
  arrow_bodies.lifetime.sec = 1;
  arrow_bodies.type = beluga_ros::msg::Marker::LINE_LIST;
  arrow_bodies.action = beluga_ros::msg::Marker::ADD;
  arrow_bodies.points.reserve(histogram.size() * 2);  // 2 vertices per arrow body
  arrow_bodies.colors.reserve(histogram.size() * 2);

  auto& arrow_heads = message.markers.emplace_back();
  arrow_heads.id = 1;
  arrow_heads.ns = "heads";
  arrow_heads.scale.x = 1.0;
  arrow_heads.scale.y = 1.0;
  arrow_heads.scale.z = 1.0;
  arrow_heads.color.a = 1.0;
  arrow_heads.pose.orientation.w = 1.0;
  arrow_heads.lifetime.sec = 1;
  arrow_heads.type = beluga_ros::msg::Marker::TRIANGLE_LIST;
  arrow_heads.action = beluga_ros::msg::Marker::ADD;
  arrow_heads.points.reserve(histogram.size() * 3);  // 3 vertices per arrow head
  arrow_heads.colors.reserve(histogram.size() * 3);

  auto min_scale_factor = Weight{1.0};
  for (const auto& [state, weight] : histogram) {
    // fix markers scale ratio to ensure they can still be seen
    using std::max;
    const auto scale_factor = max(weight / max_bin_weight, 1e-1);  // or 1:10
    if (scale_factor < min_scale_factor) {
      min_scale_factor = scale_factor;
    }

    // use an inverted rainbow scale of decreasing opacity: bright red
    // for the most likely state and dim purple for the least likely one
    const auto vertex_color = detail::alphaHueToRGBA(
        static_cast<float>((1.0 - scale_factor) * 270.0), static_cast<float>(0.25 + 0.75 * scale_factor));

    // linearly scale arrow sizes using particle weights too
    arrow_bodies.points.push_back(tf2::toMsg(state * (scale_factor * arrow_body_base)));
    arrow_bodies.points.push_back(tf2::toMsg(state * (scale_factor * arrow_head_base)));
    arrow_bodies.colors.insert(arrow_bodies.colors.end(), 2, vertex_color);

    arrow_heads.points.push_back(tf2::toMsg(state * (scale_factor * arrow_head_left_corner)));
    arrow_heads.points.push_back(tf2::toMsg(state * (scale_factor * arrow_head_right_corner)));
    arrow_heads.points.push_back(tf2::toMsg(state * (scale_factor * arrow_head_tip)));

    arrow_heads.colors.insert(arrow_heads.colors.end(), 3, vertex_color);
  }

  arrow_bodies.scale.x = static_cast<double>(min_scale_factor * kArrowHeadWidth) * 0.8;

  return message;
}

/// Assign a pose distribution to a markers message for visualization with suitable resolutions.
/**
 * \param particles Pose distribution, as a particle cloud itself.
 * \param[out] message Markers message to be assigned.
 * \tparam Particles A sized range type whose value type satisfies \ref ParticlePage "Particle" named requirements.
 */
template <
    class Particles,
    class Particle = ranges::range_value_t<Particles>,
    class State = typename beluga::state_t<Particle>,
    class Scalar = typename State::Scalar>
beluga_ros::msg::MarkerArray& assign_particle_cloud(Particles&& particles, beluga_ros::msg::MarkerArray& message) {
  constexpr auto kDefaultLinearResolution = Scalar{1e-3};   // ie. 1 mm
  constexpr auto kDefaultAngularResolution = Scalar{1e-3};  // ie. 0.05 degrees
  return assign_particle_cloud(
      std::forward<Particles>(particles), kDefaultLinearResolution, kDefaultAngularResolution, message);
}

}  // namespace beluga_ros

#endif  // BELUGA_ROS_PARTICLE_CLOUD_HPP

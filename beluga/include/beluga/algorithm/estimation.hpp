// Copyright 2022-2023 Ekumen, Inc.
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

#ifndef BELUGA_ALGORITHM_ESTIMATION_HPP
#define BELUGA_ALGORITHM_ESTIMATION_HPP

#include <range/v3/range/access.hpp>
#include <range/v3/range/primitives.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/repeat_n.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/types.hpp>

#include <numeric>

/**
 * \file
 * \brief Implementation of algorithms that allow calculating the estimated state of
 *  a particle filter.
 */

namespace beluga {

/// Calculates the covariance of a range given its mean and the weights of each element.
/**
 * \tparam Range A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range) type whose
 *  value type is `Sophus::Vector2<Scalar>`.
 * \tparam WeightsRange A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range) type whose
 *  value type is `Scalar`.
 * \tparam Scalar A scalar type, e.g. double or float.
 * \param range Range to be used to calculate the covariance.
 * \param normalized_weights Range with the normalized (total weight 1.0) weights of the samples in 'range'.
 * \param mean The previously calculated mean of range. The value must be correct for the resulting
 *  covariance to be correct.
 * \return The calculated covariance, as a `Sophus::Matrix2<Scalar>`.
 */
template <class Range, class WeightsRange, class Scalar>
Sophus::Matrix2<Scalar>
calculate_covariance(Range&& range, WeightsRange&& normalized_weights, const Sophus::Vector2<Scalar>& mean) {
  auto translations_view = range | ranges::views::common;
  auto normalized_weights_view = normalized_weights | ranges::views::common;

  auto calculate_weighted_sample_covariance = [mean](const auto& value, const auto& weight) {
    const auto centered = value - mean;
    return Sophus::Vector3<Scalar>{
        weight * centered.x() * centered.x(),  // weighted sample x autocovariance
        weight * centered.x() * centered.y(),  // weighted sample xy cross-covariance
        weight * centered.y() * centered.y(),  // weighted sample y autocovariance
    };
  };

  // calculate the averaging factor for the weighted covariance estimate
  // See https://en.wikipedia.org/wiki/Sample_mean_and_covariance#Weighted_samples
  const auto squared_weight_sum = std::transform_reduce(
      normalized_weights_view.begin(), normalized_weights_view.end(), Scalar{0.0}, std::plus{},
      [](const auto& weight) { return (weight * weight); });

  // calculate the x autocovariance, xy cross-covariance, and y autocovariance
  Sophus::Vector3<Scalar> coefficients =
      std::transform_reduce(
          translations_view.begin(), translations_view.end(), normalized_weights_view.begin(),
          Sophus::Vector3<Scalar>::Zero().eval(), std::plus{}, calculate_weighted_sample_covariance) /
      (1.0 - squared_weight_sum);

  // create the symmetric 2x2 translation covariance matrix from the coefficients
  auto covariance_matrix = Sophus::Matrix2<Scalar>{};
  covariance_matrix << coefficients(0), coefficients(1), coefficients(1), coefficients(2);
  return covariance_matrix;
}

/// Convenience overload that calculates the covariance of a range given its mean for the case where all
/// samples have the same weight.
/**
 * \tparam Range A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range) type whose
 *  value type is `Sophus::Vector2<Scalar>`.
 * \tparam Scalar A scalar type, e.g. double or float.
 * \param range Range to be used to calculate the covariance.
 * \param mean The previously calculated mean of range. The value must be correct for the resulting
 *  covariance to be correct.
 * \return The calculated covariance, as a `Sophus::Matrix2<Scalar>`.
 */
template <class Range, class Scalar>
Sophus::Matrix2<Scalar> calculate_covariance(Range&& range, const Sophus::Vector2<Scalar>& mean) {
  const auto sample_count = range.size();
  return calculate_covariance(
      range,
      ranges::views::repeat_n(1.0 / static_cast<Scalar>(sample_count), static_cast<std::ptrdiff_t>(sample_count)),
      mean);
}

/// Returns a pair consisting of the estimated mean pose and its covariance.
/**
 * Given a range of poses, computes the estimated pose by averaging the translation
 * and rotation parts.
 * Computes the covariance matrix of the translation parts and the circular variance
 * of the rotation angles to create a 3x3 covariance matrix.
 * It does not take into account the particle weights.
 *
 * \tparam Poses A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range) type whose
 *  value type is `Sophus::SE2<Scalar>`.
 * \tparam Weights A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range) type whose
 *  value type is `Scalar`.
 * \tparam Pose The pose value type of the given range.
 * \tparam Scalar A scalar type, e.g. double or float.
 * \param poses Poses of equally weighted 2D poses.
 * \param weights Poses of equally weighted 2D poses.
 * \return The estimated pose and its 3x3 covariance matrix.
 */
template <
    class Poses,
    class Weights,
    class Pose = ranges::range_value_t<Poses>,
    class Scalar = typename Pose::Scalar,
    typename = std::enable_if_t<std::is_same_v<Pose, typename Sophus::SE2<Scalar>>>>
std::pair<Sophus::SE2<Scalar>, Sophus::Matrix3<Scalar>> estimate(Poses&& poses, Weights&& weights) {
  auto translation_view = poses | ranges::views::transform([](const auto& pose) { return pose.translation(); });
  auto weights_view = weights | ranges::views::common;
  const auto weights_sum = std::accumulate(weights_view.begin(), weights_view.end(), 0.0);
  auto normalized_weights_view =
      weights_view | ranges::views::transform([weights_sum](const auto& weight) { return weight / weights_sum; });

  // map sophus pose 2D pose into a 4D Eigen vector. Mapping exploits that Sophus stores the 2D transform
  // as two elements for the linear translation, and two more for the orientation (in complex number form)
  const auto pose_to_weighted_eigen_vector = [](const auto& pose, const auto& weight) {
    return Eigen::Map<const Sophus::Vector4<Scalar>>{pose.data()} * weight;
  };

  // Compute the average of all the coefficients of the SE2 group elements and construct a new SE2 element. Notice
  // that after averaging the complex representation of the orientation the resulting complex is not on the unit circle.
  // This is expected and the value will be renormalized after having used the non-normal result to estimate the
  // orientation autocovariance.
  const Sophus::Vector4<Scalar> mean_pose_vector = std::transform_reduce(
      poses.begin(), poses.end(), normalized_weights_view.begin(), Sophus::Vector4<Scalar>::Zero().eval(), std::plus{},
      pose_to_weighted_eigen_vector);

  // Calculate the weighted pose estimation
  Sophus::SE2<Scalar> estimated_pose = Eigen::Map<const Sophus::SE2<Scalar>>{mean_pose_vector.data()};

  Sophus::Matrix3<Scalar> covariance_matrix = Sophus::Matrix3<Scalar>::Zero();

  // Compute the covariance of the translation part.
  covariance_matrix.template topLeftCorner<2, 2>() =
      calculate_covariance(translation_view, normalized_weights_view, estimated_pose.translation());

  // Compute the orientation variance and re-normalize the rotation component.
  if (estimated_pose.so2().unit_complex().norm() < std::numeric_limits<double>::epsilon()) {
    // Handle the case where both averages are too close to zero.
    // Return zero yaw and infinite variance.
    covariance_matrix.coeffRef(2, 2) = std::numeric_limits<double>::infinity();
    estimated_pose.so2() = Sophus::SO2<Scalar>{0.0};
  } else {
    // See circular standard deviation in
    // https://en.wikipedia.org/wiki/Directional_statistics#Dispersion.
    covariance_matrix.coeffRef(2, 2) = -2.0 * std::log(estimated_pose.so2().unit_complex().norm());
    estimated_pose.so2().normalize();
  }
  return std::pair{estimated_pose, covariance_matrix};
}

/// Computes mean and covariance Returns a pair consisting of the estimated mean pose and its covariance.
/**
 * Given a range of 2D poses, computes the estimated pose by averaging the translation
 * and rotation parts, assuming all poses are equally weighted.
 * Computes the covariance matrix of the translation parts and the circular variance
 * of the rotation angles to create a 3x3 covariance matrix.
 * It does not take into account the particle weights. This is appropriate for use with
 * filter update cycles that resample the particle set at every iteration, since
 * in that case the belief is fully represented by the spatial distribution of the
 * particles, and the particle weights provide no additional information.
 *
 * \tparam Poses A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range) type whose
 *  value type is `Sophus::SE2<Scalar>`.
 * \tparam Pose The pose value type of the given range.
 * \tparam Scalar A scalar type, e.g. double or float.
 * \param poses Poses of equally weighted 2D poses.
 * \return The estimated pose and its 3x3 covariance matrix.
 */
template <
    class Poses,
    class Pose = ranges::range_value_t<Poses>,
    class Scalar = typename Pose::Scalar,
    typename = std::enable_if_t<std::is_same_v<Pose, typename Sophus::SE2<Scalar>>>>
std::pair<Sophus::SE2<Scalar>, Sophus::Matrix3<Scalar>> estimate(Poses&& poses) {
  return beluga::estimate(poses, ranges::views::repeat_n(1.0, static_cast<std::ptrdiff_t>(poses.size())));
}

}  // namespace beluga

#endif

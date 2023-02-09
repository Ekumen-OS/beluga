// Copyright 2022 Ekumen, Inc.
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

#include <beluga/type_traits.hpp>
#include <range/v3/range/access.hpp>
#include <range/v3/range/primitives.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>

/**
 * \file
 * \brief Implementation of algorithms that allow calculating the estimated state of
 *  a particle filter.
 */

namespace beluga {

/// Calculates the covariance of a range given its mean.
/**
 * \tparam Range A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range) type whose
 *  `Range::value_type` is `Eigen::Vector2<Scalar>`.
 * \tparam Scalar A scalar type, e.g. double or float.
 * \param range Range to be used to calculate the covariance.
 * \param mean The previously calculated mean of range. The value must be correct for the resulting
 *  covariance to be correct.
 * \return The calculated covariance, as a `Eigen::Matrix2<Scalar>`.
 */
template <class Range, class Scalar>
Eigen::Matrix2<Scalar> covariance(const Range& range, const Eigen::Vector2<Scalar>& mean) {
  Eigen::Vector3<Scalar> coefficients = std::transform_reduce(
      ranges::begin(range), ranges::end(range), Eigen::Vector3<Scalar>::Zero().eval(), std::plus{},
      [mean](const auto& value) {
        const auto centered = value - mean;
        return Eigen::Vector3<Scalar>{
            centered.x() * centered.x(),
            centered.x() * centered.y(),
            centered.y() * centered.y(),
        };
      });
  coefficients /= (static_cast<Scalar>(ranges::size(range) - 1));
  auto covariance_matrix = Eigen::Matrix2<Scalar>{};
  covariance_matrix << coefficients(0), coefficients(1), coefficients(1), coefficients(2);
  return covariance_matrix;
}

/// Returns a pair consisting of the estimated mean pose and its covariance.
/**
 * Given a range of poses, computes the estimated pose by averaging the translation
 * and rotation parts.
 * Computes the covariance matrix of the translation parts and the circular variance
 * of the rotation angles to create a 3x3 covariance matrix.
 * It does not take into account the particle weights.
 *
 * \tparam Range A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range) type whose
 *  `Range::value_type` is `Sophus::SE2d<Scalar>`.
 * \tparam Pose The pose value type of the given range.
 * \tparam Scalar A scalar type, e.g. double or float.
 * \param poses Range of equally weighted 2D poses.
 * \return The estimated pose and its 3x3 covariance matrix.
 */
template <
    class Range,
    class Pose = typename ranges::range_value_t<Range>,
    class Scalar = typename Pose::Scalar,
    typename = std::enable_if_t<std::is_same_v<Pose, typename Sophus::SE2<Scalar>>>>
std::pair<Sophus::SE2<Scalar>, Eigen::Matrix3<Scalar>> estimate(const Range& poses) {
  const auto translation_view = poses | ranges::views::transform([](const auto& pose) { return pose.translation(); });

  // Compute the average of all the coefficients of the SE(2) group elements and
  // construct a new SE(2) element.
  const Eigen::Vector4<Scalar> sum_vector = std::transform_reduce(
      poses.begin(), poses.end(), Eigen::Vector4<Scalar>::Zero().eval(), std::plus{},
      [](const auto& pose) { return Eigen::Map<const Eigen::Vector4d>{pose.data()}; });
  const Eigen::Vector4<Scalar> mean_vector = sum_vector / static_cast<double>(poses.size());
  Sophus::SE2<Scalar> mean = Eigen::Map<const Sophus::SE2<Scalar>>{mean_vector.data()};

  // Compute the covariance of the translation part.
  Eigen::Matrix3<Scalar> covariance_matrix = Eigen::Matrix3<Scalar>::Zero();
  covariance_matrix.template topLeftCorner<2, 2>() = covariance(translation_view, mean.translation());

  // Compute the circular variance and re-normalize the rotation component.
  if (mean.so2().unit_complex().norm() < std::numeric_limits<double>::epsilon()) {
    // Handle the case where both averages are too close to zero.
    // Return zero yaw and infinite variance.
    covariance_matrix.coeffRef(2, 2) = std::numeric_limits<double>::infinity();
    mean.so2() = Sophus::SO2<Scalar>{0.0};
  } else {
    // See circular standard deviation in
    // https://en.wikipedia.org/wiki/Directional_statistics#Dispersion.
    covariance_matrix.coeffRef(2, 2) = -2.0 * std::log(mean.so2().unit_complex().norm());
    mean.so2().normalize();
  }
  return std::pair{mean, covariance_matrix};
}

}  // namespace beluga

#endif

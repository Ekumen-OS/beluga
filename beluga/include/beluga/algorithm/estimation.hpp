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

/**
 * \page StateEstimatorPage Beluga named requirements: StateEstimator
 * The requirements that a state estimator must satisfy.
 *
 * \section StateEstimationRequirements Requirements
 * `T` is a `StateEstimator` if given a (possibly const) instance `p` of `T`, the following is satisfied:
 * - `p.estimate()` is a valid expression.
 * - `std::get<0>(p.estimate())` is valid.
 *   `decltype(std::get<0>(p.estimate()))` represents the estimated state.
 * - `std::get<1>(p.estimate())` is valid.
 *   `decltype(std::get<1>(p.estimate()))` represents the covariance of the estimation.
 *
 * \section StateEstimationLinks See also
 * - beluga::SimpleStateEstimator<Mixin, Sophus::SE2d>
 */

namespace beluga {

/// Calculates the covariance of a range given its mean.
/**
 * \tparam Range A [sized range](https://en.cppreference.com/w/cpp/ranges/sized_range) type whose
 *  value type is `Eigen::Vector2<Scalar>`.
 * \tparam Scalar A scalar type, e.g. double or float.
 * \param range Range to be used to calculate the covariance.
 * \param mean The previously calculated mean of range. The value must be correct for the resulting
 *  covariance to be correct.
 * \return The calculated covariance, as a `Eigen::Matrix2<Scalar>`.
 */
template <class Range, class Scalar>
Eigen::Matrix2<Scalar> covariance(Range&& range, const Eigen::Vector2<Scalar>& mean) {
  auto view = range | ranges::views::common;
  Eigen::Vector3<Scalar> coefficients = std::transform_reduce(
      view.begin(), view.end(), Eigen::Vector3<Scalar>::Zero().eval(), std::plus{}, [mean](const auto& value) {
        const auto centered = value - mean;
        return Eigen::Vector3<Scalar>{
            centered.x() * centered.x(),
            centered.x() * centered.y(),
            centered.y() * centered.y(),
        };
      });
  coefficients /= (static_cast<Scalar>(view.size() - 1));
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
 *  value type is `Sophus::SE2<Scalar>`.
 * \tparam Pose The pose value type of the given range.
 * \tparam Scalar A scalar type, e.g. double or float.
 * \param range Range of equally weighted 2D poses.
 * \return The estimated pose and its 3x3 covariance matrix.
 */
template <
    class Range,
    class Pose = ranges::range_value_t<Range>,
    class Scalar = typename Pose::Scalar,
    typename = std::enable_if_t<std::is_same_v<Pose, typename Sophus::SE2<Scalar>>>>
std::pair<Sophus::SE2<Scalar>, Eigen::Matrix3<Scalar>> estimate(Range&& range) {
  auto pose_view = range | ranges::views::common;
  auto translation_view = range | ranges::views::transform([](const auto& pose) { return pose.translation(); });

  // Compute the average of all the coefficients of the SE(2) group elements and
  // construct a new SE(2) element.
  const Eigen::Vector4<Scalar> sum_vector = std::transform_reduce(
      pose_view.begin(), pose_view.end(), Eigen::Vector4<Scalar>::Zero().eval(), std::plus{},
      [](const auto& pose) { return Eigen::Map<const Eigen::Vector4<Scalar>>{pose.data()}; });
  const Eigen::Vector4<Scalar> mean_vector = sum_vector / static_cast<double>(pose_view.size());
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

/// Pure abstract class representing the estimation interface.
struct EstimationInterface2d {
  /// Virtual destructor.
  virtual ~EstimationInterface2d() = default;

  /// Returns the estimate state of the particle filter.
  /**
   * \return The estimated 2D pose and its 3x3 covariance matrix.
   */
  [[nodiscard]] virtual std::pair<Sophus::SE2d, Eigen::Matrix3d> estimate() const = 0;
};

/// Primary template for a simple state estimator.
template <class Mixin, class State>
class SimpleStateEstimator;

/// Partial template specialization for simple state estimator in 2D.
/**
 * This class implements the EstimationInterface2d interface
 * and satisfies \ref StateEstimatorPage.
 *
 * It is an estimator that calculates the pose mean and covariance using all the particles.
 */
template <class Mixin>
class SimpleStateEstimator<Mixin, Sophus::SE2d> : public Mixin {
 public:
  /// Constructs a SimpleStateEstimator instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit SimpleStateEstimator(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// \copydoc EstimationInterface2d::estimate()
  [[nodiscard]] std::pair<Sophus::SE2d, Eigen::Matrix3d> estimate() const final {
    return beluga::estimate(this->self().states());
  }
};

/// An alias template for the simple state estimator in 2D.
template <class Mixin>
using SimpleStateEstimator2d = SimpleStateEstimator<Mixin, Sophus::SE2d>;

}  // namespace beluga

#endif

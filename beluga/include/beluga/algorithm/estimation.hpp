// Copyright 2022-2024 Ekumen, Inc.
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

#include <range/v3/algorithm/count_if.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/repeat_n.hpp>
#include <range/v3/view/transform.hpp>

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <sophus/types.hpp>

#include <numeric>
#include <type_traits>

/**
 * \file
 * \brief Implementation of algorithms that allow calculating the estimated state of a particle filter.
 */

namespace beluga {

namespace detail {

/// \cond detail

struct mean_fn {
  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Value = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>,
      std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Value>, Value>, int> = 0>
  auto operator()(Values&& values, Weights&& normalized_weights, Projection projection = {}) const {
    static_assert(ranges::input_range<Values>);
    static_assert(ranges::input_range<Weights>);

    constexpr int M = Value::RowsAtCompileTime;
    constexpr int N = Value::ColsAtCompileTime;
    static_assert(N == 1);
    using Scalar = typename Value::Scalar;
    auto accumulator = Sophus::Vector<Scalar, M>::Zero().eval();

    auto it = ranges::begin(values);
    const auto last = ranges::end(values);
    auto weights_it = ranges::begin(normalized_weights);

    assert(it != last);
    assert(weights_it != ranges::end(normalized_weights));

    for (; it != last; ++weights_it, ++it) {
      accumulator += *weights_it * projection(*it);
    }

    assert(weights_it == ranges::end(normalized_weights));

    return accumulator;
  }

  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Scalar = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>,
      std::enable_if_t<std::is_floating_point_v<Scalar>, int> = 0>
  auto operator()(Values&& values, Weights&& normalized_weights, Projection projection = {}) const -> Scalar {
    return (*this)(
        std::forward<Values>(values), std::forward<Weights>(normalized_weights),
        [projection = std::move(projection)](const auto& value) {
          return Sophus::Vector<Scalar, 1>(projection(value));
        })(0);
  }

  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Value = std::decay_t<ranges::range_value_t<Values>>,
      class Scalar = typename Value::Scalar,
      std::enable_if_t<std::is_base_of_v<Sophus::SO2Base<Value>, Value>, int> = 0>
  auto operator()(Values&&, Weights&&, Projection = {}) const -> Value = delete;  // not-implemented

  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Value = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>,
      class Scalar = typename Value::Scalar,
      class = std::enable_if_t<std::is_base_of_v<Sophus::SE2Base<Value>, Value>>>
  auto operator()(Values&& values, Weights&& normalized_weights, Projection projection = {}) const
      -> Sophus::SE2<Scalar> {
    // Compute the average of all the coefficients of the SE2 group elements and construct a new SE2 element. Notice
    // that after averaging the complex representation of the orientation the resulting complex is not on the unit
    // circle. This is expected and the value will be renormalized before returning.
    Sophus::Vector4<Scalar> mean_vector = (*this)(
        std::forward<Values>(values), std::forward<Weights>(normalized_weights),
        [projection = std::move(projection)](const auto& value) {
          return Eigen::Map<const Sophus::Vector4<Scalar>>{projection(value).data()};
        });

    Eigen::Map<Sophus::SE2<Scalar>> mean{mean_vector.data()};
    mean.so2().normalize();
    return mean;
  }

  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Value = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>,
      class Scalar = typename Value::Scalar,
      std::enable_if_t<std::is_base_of_v<Eigen::QuaternionBase<Value>, Value>, int> = 0>
  auto operator()(Values&& values, Weights&& normalized_weights, Projection projection = {}) const
      -> Eigen::Quaternion<Scalar> {
    static_assert(ranges::input_range<Values>);
    static_assert(ranges::sized_range<Values>);

    static_assert(ranges::input_range<Weights>);
    static_assert(ranges::sized_range<Weights>);

    // This implementation is based on Sophus' average methods with the variant of non-uniform weights.
    // See https://github.com/strasdat/Sophus/blob/d0b7315a0d90fc6143defa54596a3a95d9fa10ec/sophus/average.hpp#L135
    // See https://ntrs.nasa.gov/api/citations/20070017872/downloads/20070017872.pdf equation (13).

    const auto size = static_cast<int>(ranges::size(values));
    auto matrix = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>(4, size);

    auto weights_it = ranges::begin(normalized_weights);
    auto it = ranges::begin(values);

    assert(it != ranges::end(values));
    assert(weights_it != ranges::end(normalized_weights));

    for (int index = 0; index < size; ++index, ++weights_it, ++it) {
      matrix.col(index) = *weights_it * projection(*it).coeffs();
    }

    assert(it == ranges::end(values));
    assert(weights_it == ranges::end(normalized_weights));

    const auto solver = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 4, 4>>{matrix * matrix.transpose()};
    assert(solver.info() == Eigen::Success);

    // This is not the same as `result{solver.eigenvectors().col(3).real()}`.
    // Eigen's internal coefficient order is different from the constructor one.
    // Eigenvalues are sorted in increasing order, so eigenvalue number 3 is the max.
    Eigen::Quaternion<Scalar> result;
    result.coeffs() << solver.eigenvectors().col(3).real();
    return result;
  }

  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Value = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>,
      class Scalar = typename Value::Scalar,
      std::enable_if_t<std::is_base_of_v<Sophus::SO3Base<Value>, Value>, int> = 0>
  auto operator()(Values&& values, Weights&& normalized_weights, Projection projection = {}) const
      -> Sophus::SO3<Scalar> {
    return {(*this)(
        std::forward<Values>(values), std::forward<Weights>(normalized_weights),
        [projection = std::move(projection)](const auto& value) { return projection(value).unit_quaternion(); })};
  }

  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Value = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>,
      class Scalar = typename Value::Scalar,
      std::enable_if_t<std::is_base_of_v<Sophus::SE3Base<Value>, Value>, int> = 0>
  auto operator()(Values&& values, Weights&& normalized_weights, Projection projection = {}) const
      -> Sophus::SE3<Scalar> {
    static_assert(ranges::forward_range<Values>);   // must allow multi-pass
    static_assert(ranges::forward_range<Weights>);  // must allow multi-pass

    return {
        (*this)(values, normalized_weights, [&projection](const auto& v) { return projection(v).unit_quaternion(); }),
        (*this)(values, normalized_weights, [&projection](const auto& v) { return projection(v).translation(); })};
  }

  template <
      class Values,
      class Projection = ranges::identity,
      class Value = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>>
  auto operator()(Values&& values, Projection projection = {}) const {
    using Scalar = typename Value::Scalar;
    static_assert(ranges::sized_range<Values>);
    const auto size = ranges::size(values);
    auto weights = ranges::views::repeat_n(1.0 / static_cast<Scalar>(size), static_cast<std::ptrdiff_t>(size));
    return (*this)(std::forward<Values>(values), std::move(weights), std::move(projection));
  }
};

/// \endcond

}  // namespace detail

/// Calculate the weighted mean (or average) of a range of values.
/**
 * The inputs are a range of values, a range of corresponding weights, and an optional projection function
 * to convert to the right value-type.
 *
 * It supports floating-point numbers, vectors, quaternions, and Lie group elements like SE2 and SE3.
 *
 * The weights are assumed to already be normalized. Non-normalized weights will yield incorrect results.
 */
inline constexpr detail::mean_fn mean;

namespace detail {

/// \cond detail

struct covariance_fn {
  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Value = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>,
      std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Value>, Value>, int> = 0>
  auto operator()(
      Values&& values,
      Weights&& normalized_weights,
      const typename Value::PlainMatrix& mean,
      Projection projection = {}) const {
    static_assert(ranges::input_range<Values>);
    static_assert(ranges::input_range<Weights>);

    constexpr int M = Value::RowsAtCompileTime;
    constexpr int N = Value::ColsAtCompileTime;
    static_assert(N == 1);
    using Scalar = typename Value::Scalar;
    auto accumulator = Eigen::Matrix<Scalar, M, M>::Zero().eval();
    auto squared_weight_sum = Scalar{0.0};

    auto it = ranges::begin(values);
    const auto last = ranges::end(values);
    auto weights_it = ranges::begin(normalized_weights);

    assert(it != last);
    assert(weights_it != ranges::end(normalized_weights));

    for (; it != last; ++weights_it, ++it) {
      const auto& value = projection(*it);
      const auto weight = *weights_it;
      const auto centered = value - mean;
      accumulator.noalias() += weight * centered * centered.transpose();
      squared_weight_sum += weight * weight;
    }

    assert(weights_it == ranges::end(normalized_weights));
    assert(squared_weight_sum < 1.0);

    accumulator /= (1.0 - squared_weight_sum);
    return accumulator;
  }

  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Value = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>,
      std::enable_if_t<std::is_floating_point_v<Value>, int> = 0>
  auto operator()(Values&& values, Weights&& normalized_weights, Value mean, Projection projection = {}) const {
    static_assert(ranges::input_range<Values>);
    static_assert(ranges::input_range<Weights>);

    auto accumulator = Value{0};
    auto non_zero_weight_count = Value{0};

    auto it = ranges::begin(values);
    const auto last = ranges::end(values);
    auto weights_it = ranges::begin(normalized_weights);

    assert(it != last);
    assert(weights_it != ranges::end(normalized_weights));

    for (; it != last; ++weights_it, ++it) {
      const auto& value = projection(*it);
      const auto weight = *weights_it;
      const auto centered = value - mean;
      accumulator += weight * centered * centered;

      if (weight != 0) {
        non_zero_weight_count += 1;
      }
    }

    assert(weights_it == ranges::end(normalized_weights));
    assert(non_zero_weight_count > 1);

    accumulator *= non_zero_weight_count / (non_zero_weight_count - 1);
    return accumulator;
  }

  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Value = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>,
      std::enable_if_t<
          std::disjunction_v<
              std::is_base_of<Sophus::SE2Base<Value>, Value>,
              std::is_base_of<Sophus::SE3Base<Value>, Value>>,
          int> = 0>
  auto operator()(Values&& values, Weights&& normalized_weights, const Value& mean, Projection projection = {}) const {
    static_assert(ranges::input_range<Values>);
    static_assert(ranges::input_range<Weights>);

    // For SE3 estimates, we represent the estimate as a noiseless pose and covariance in se3,
    // the tangent space of the SE3 manifold.
    //
    // Users may perform the appropriate conversions to get the covariance matrix into their parametrization of
    // interest.
    //
    // This reasoning is based on "Characterizing the Uncertainty of Jointly Distributed Poses in the Lie
    // Algebra" by Mangelson et al. (https://robots.et.byu.edu/jmangelson/pubs/2020/mangelson2020tro.pdf).
    //
    // See section III. E) "Defining Random Variables over Poses" for more context.
    auto accumulator = Value::Adjoint::Zero().eval();
    auto squared_weight_sum = typename Value::Scalar{0.0};

    auto it = ranges::begin(values);
    const auto last = ranges::end(values);
    auto weights_it = ranges::begin(normalized_weights);

    assert(it != last);
    assert(weights_it != ranges::end(normalized_weights));

    const auto inverse_mean = mean.inverse();

    for (; it != last; ++weights_it, ++it) {
      const auto& value = projection(*it);
      const auto weight = *weights_it;
      const auto centered = (inverse_mean * value).log();
      accumulator.noalias() += weight * centered * centered.transpose();
      squared_weight_sum += weight * weight;
    }

    assert(weights_it == ranges::end(normalized_weights));
    assert(squared_weight_sum < 1.0);

    accumulator /= (1.0 - squared_weight_sum);
    return accumulator;
  }

  template <
      class Values,
      class Weights,
      class Projection = ranges::identity,
      class Value = std::decay_t<ranges::range_value_t<Values>>,
      std::enable_if_t<
          std::disjunction_v<
              std::is_base_of<Eigen::QuaternionBase<Value>, Value>,
              std::is_base_of<Sophus::SO2Base<Value>, Value>,
              std::is_base_of<Sophus::SO3Base<Value>, Value>>,
          int> = 0>
  auto operator()(Values&&, Weights&&, const Value&, Projection = {}) const -> Value = delete;  // not-implemented

  template <
      class Values,
      class Projection = ranges::identity,
      class Value = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>>
  auto operator()(Values&& values, const Value& mean, Projection projection = {}) const {
    static_assert(ranges::sized_range<Values>);
    const auto size = ranges::size(values);
    auto weights = ranges::views::repeat_n(1.0 / static_cast<double>(size), static_cast<std::ptrdiff_t>(size));
    return (*this)(std::forward<Values>(values), std::move(weights), mean, std::move(projection));
  }
};

/// \endcond

}  // namespace detail

/// Calculate the weighted covariance of a range of values.
/**
 * The inputs are a range of values, a range of corresponding weights, the pre-computed mean, and an optional projection
 * function to convert to the right value-type.
 *
 * It supports floating-point numbers, vectors, and Lie group elements like SE2 and SE3.
 *
 * For Lie group elements, the function computes covariance in the tangent space, representing the uncertainty of poses
 * on the Lie manifold. Users may perform the appropriate conversions to get the covariance matrix into their
 * parametrization of interest.
 *
 * The weights are assumed to already be normalized. Non-normalized weights will yield incorrect results.
 */
inline constexpr detail::covariance_fn covariance;

namespace detail {

/// \cond detail

struct estimate_fn {
  template <
      class Values,
      class Weights,
      class Value = std::decay_t<ranges::range_value_t<Values>>,
      class = std::enable_if_t<std::disjunction_v<
          std::is_floating_point<Value>,
          std::is_base_of<Eigen::MatrixBase<Value>, Value>,
          std::is_base_of<Sophus::SE3Base<Value>, Value>>>>
  auto operator()(Values&& values, Weights&& weights) const {
    static_assert(ranges::forward_range<Values>);   // must allow multi-pass
    static_assert(ranges::forward_range<Weights>);  // must allow multi-pass

    auto normalized_weights = weights | ranges::views::transform([sum = ranges::accumulate(weights, 0.0)](auto weight) {
                                return weight / sum;
                              });

    const auto mean = beluga::mean(values, normalized_weights);
    const auto variance = beluga::covariance(values, normalized_weights, mean);
    return std::make_pair(mean, variance);
  }

  template <
      class Values,
      class Weights,
      class Value = std::decay_t<ranges::range_value_t<Values>>,
      class Scalar = typename Value::Scalar,
      class = std::enable_if_t<std::is_base_of_v<Sophus::SE2Base<Value>, Value>>>
  auto operator()(Values&& values, Weights&& weights) const -> std::pair<Sophus::SE2<Scalar>, Sophus::Matrix3<Scalar>> {
    static_assert(ranges::forward_range<Values>);   // must allow multi-pass
    static_assert(ranges::forward_range<Weights>);  // must allow multi-pass

    auto normalized_weights = weights | ranges::views::transform([sum = ranges::accumulate(weights, 0.0)](auto weight) {
                                return weight / sum;
                              });

    // Compute the average of all the coefficients of the SE2 group elements and construct a new SE2 element. Notice
    // that after averaging the complex representation of the orientation the resulting complex is not on the unit
    // circle. This is expected and the value will be renormalized after having used the non-normal result to estimate
    // the orientation autocovariance.
    const Sophus::Vector4<Scalar> mean_vector = beluga::mean(values, normalized_weights, [](const auto& value) {
      return Eigen::Map<const Sophus::Vector4<Scalar>>{value.data()};
    });

    auto mean = Sophus::SE2<Scalar>{Eigen::Map<const Sophus::SE2<Scalar>>{mean_vector.data()}};
    auto covariance = Sophus::Matrix3<Scalar>::Zero().eval();

    // Compute the covariance of the translation part.
    covariance.template topLeftCorner<2, 2>() = beluga::covariance(
        values, normalized_weights, mean.translation(), [](const auto& value) { return value.translation(); });

    // Compute the orientation variance and re-normalize the rotation component (after using the non-normal result).
    if (mean.so2().unit_complex().norm() < std::numeric_limits<double>::epsilon()) {
      // Handle the case where both averages are too close to zero.
      // Return zero yaw and infinite variance.
      covariance.coeffRef(2, 2) = std::numeric_limits<double>::infinity();
      mean.so2() = Sophus::SO2<Scalar>{0.0};
      // TODO(nahuel): Consider breaking the existing API and return
      // an optional to handle degenerate cases just like Sophus does.
    } else {
      // See circular standard deviation in
      // https://en.wikipedia.org/wiki/Directional_statistics#Dispersion.
      covariance.coeffRef(2, 2) = -2.0 * std::log(mean.so2().unit_complex().norm());
      mean.so2().normalize();
    }

    return std::make_pair(mean, covariance);
  }

  template <
      class Values,
      class Weights,
      class Value = std::decay_t<ranges::range_value_t<Values>>,
      class = std::enable_if_t<std::disjunction_v<
          std::is_base_of<Eigen::QuaternionBase<Value>, Value>,
          std::is_base_of<Sophus::SO2Base<Value>, Value>,
          std::is_base_of<Sophus::SO3Base<Value>, Value>>>>
  auto operator()(Values&&, Weights&&) const -> Value = delete;  // not-implemented

  template <class Values>
  auto operator()(Values&& values) const {
    static_assert(ranges::sized_range<Values>);
    const auto size = ranges::size(values);
    auto weights = ranges::views::repeat_n(1.0 / static_cast<double>(size), static_cast<std::ptrdiff_t>(size));
    return (*this)(std::forward<Values>(values), std::move(weights));
  }
};

/// \endcond

}  // namespace detail

/// Calculate the estimate (mean and covariance) of a range of values.
/**
 * The inputs are a range of values and a range of corresponding weights.
 *
 * It supports floating-point numbers, vectors, and Lie group elements like SE2 and SE3.
 *
 * It normalizes the weights before computing the mean and covariance.
 */
inline constexpr detail::estimate_fn estimate;

}  // namespace beluga

#endif

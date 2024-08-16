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

#ifndef BELUGA_ALGORITHM_UNSCENTED_TRANSFORM_HPP
#define BELUGA_ALGORITHM_UNSCENTED_TRANSFORM_HPP

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <functional>
#include <numeric>
#include <optional>
#include <range/v3/numeric.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/range/concepts.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/any_view.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>
#include <type_traits>
#include <utility>
#include <vector>
#include "beluga/eigen_compatibility.hpp"
namespace beluga {

namespace detail {

/// Templated functor to compute weighted mean for arbitrary. Assumes weights and vector's scalar types match.
struct default_weighted_mean_fn {
  /// Operator() overload for eigen types.
  template <typename T, typename Scalar = typename T::Scalar>
  T operator()(const std::vector<T>& samples, const std::vector<Scalar>& weights) const {
    return std::transform_reduce(
        samples.begin(), samples.end(), weights.begin(), T::Zero().eval(), std::plus<>{}, std::multiplies<>{});
  }
};

}  // namespace detail

/// Object for computing a euclidean weighted mean of a vector of elements.
inline constexpr detail::default_weighted_mean_fn default_weighted_mean;

/**
 * \brief Implements the unscented transform, a mathematical function used to estimate the result of applying a given
 *  a possibly nonlinear transformation to a probability distribution that is characterized with mean and covariance.
 * See https://en.wikipedia.org/wiki/Unscented_transform for more information.
 * Also see https://arxiv.org/pdf/2104.01958 Equation (5) for more context on sigma points selection.
 *
 * \tparam MeanDerived Concrete Eigen dense type used for the mean.
 * \tparam CovarianceDerived Concrete Eigen dense type used for the covariance.
 * \tparam TransferFn Callable that maps a vector from input space to output space. Possibly non-linear.
 * \tparam TransformedT State representing the output space. Deduced from the transfer function.
 * \tparam MeanFn Callable that converts a vector of elements in the output space, and weights, to its mean in the
 * output space.
 * \tparam ResidualFn Callable that computes a residual given two elements in the output space.
 * \param mean Mean in the input space.
 * \param covariance Covariance in the input space.
 * \param transfer_fn Callable that converts a* vector from input space to output space.
 * \param kappa Parameter used for sigma points selection. A sensible default will be used if not provided.
 * \param mean_fn Callable that converts a vector of elements in the output space, and
 * weights, to its mean in the output space. A sensible default is provided for the trivial (euclidian) case.
 * \param residual_fn Callable that computes a residual given two elements in the output space.. A sensible default is
 * provided for the trivial (euclidian) case.
 * \return A pair containing (mean, estimate) for the output space.
 */
template <
    typename MeanDerived,
    typename CovarianceDerived,
    typename TransferFn,
    typename TransformedT = std::result_of_t<TransferFn(MeanDerived)>,
    typename MeanFn = detail::default_weighted_mean_fn,
    typename ResidualFn = std::minus<TransformedT>>
auto unscented_transform(
    const Eigen::MatrixBase<MeanDerived>& mean,
    const Eigen::MatrixBase<CovarianceDerived>& covariance,
    TransferFn&& transfer_fn,
    std::optional<typename MeanDerived::Scalar> kappa = std::nullopt,
    MeanFn mean_fn = default_weighted_mean,
    ResidualFn residual_fn = std::minus<TransformedT>{}) {
  using Scalar = typename MeanDerived::Scalar;
  static_assert(
      std::is_same_v<typename MeanDerived::Scalar, typename CovarianceDerived::Scalar>,
      "Mean and covariance scalar types differ.");
  static_assert(std::is_floating_point_v<Scalar>);
  static_assert(MeanDerived::ColsAtCompileTime == 1, "Mean should be a column vector.");
  static_assert(
      CovarianceDerived::ColsAtCompileTime == CovarianceDerived::RowsAtCompileTime, "Covariance matrix is not square.");
  constexpr int kNin = MeanDerived::RowsAtCompileTime;
  static_assert(kNin >= 1, "Input dimension should be greater than zero.");
  static_assert(CovarianceDerived::ColsAtCompileTime == kNin);
  const double k = kappa.value_or(std::max(kNin - 3, 0));
  assert(k >= 0);
  const Scalar w0 = static_cast<Scalar>(k) / static_cast<Scalar>(kNin + k);
  std::vector<Eigen::Vector<Scalar, kNin>> sigma_points;
  std::vector<Scalar> weights;
  sigma_points.reserve(2 * kNin + 1);
  weights.resize(2 * kNin + 1);
  sigma_points.emplace_back(mean);
  weights[0] = w0;
  const Scalar wn = 1. / (2. * (kNin + k));

  // Compute the Cholesky decomposition of the covariance matrix
  const auto llt = covariance.llt();
  assert(llt.info() == Eigen::ComputationInfo::Success);
  const Eigen::Matrix<Scalar, kNin, kNin> l_matrix = llt.matrixL();
  // Precompute sqrt{n + kaappa} * L for speed sake.
  const Eigen::Matrix<Scalar, kNin, kNin> scaled_l_matrix = std::sqrt(kNin + k) * l_matrix;

  for (int i = 0; i < kNin; ++i) {
    sigma_points.emplace_back(mean + scaled_l_matrix.col(i));
    sigma_points.emplace_back(mean - scaled_l_matrix.col(i));
  }
  std::fill(weights.begin() + 1, weights.end(), wn);

  const auto transformed_sigma_points = sigma_points | ranges::views::transform(transfer_fn) | ranges::to<std::vector>;

  constexpr int kNout = decltype(transfer_fn(mean))::RowsAtCompileTime;
  static_assert(decltype(transfer_fn(mean))::ColsAtCompileTime == 1, "Output mean should be a column vector");

  const Eigen::Vector<Scalar, kNout> out_mean = mean_fn(transformed_sigma_points, weights);

  const Eigen::Matrix<Scalar, kNout, kNout> out_cov = std::transform_reduce(
      transformed_sigma_points.begin(), transformed_sigma_points.end(), weights.begin(),
      Eigen::Matrix<Scalar, kNout, kNout>::Zero().eval(), std::plus<>{},
      [&](const auto& sigma_point, const auto weight) {
        const Eigen::Vector<Scalar, kNout> error = residual_fn(sigma_point, out_mean);
        return (weight * (error * error.transpose())).eval();
      });

  return std::make_pair(out_mean, out_cov);
}

}  // namespace beluga

#endif

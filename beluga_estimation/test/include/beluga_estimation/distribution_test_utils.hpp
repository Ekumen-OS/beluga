// Copyright 2026 Ekumen, Inc.
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

#pragma once

#include <vector>

#include <Eigen/Core>

#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/distribution.hpp>

namespace beluga_estimation::testing {

// Test tolerance constants
constexpr double kTolerance = 1e-6;
constexpr double kStatTolerance = 0.05;
constexpr std::size_t kNumSamples = 100'000;

/**
 * @brief Draws samples from a distribution via the virtual interface.
 *
 * @tparam Dist The distribution type.
 * @tparam Scalar The scalar type of the sample vectors.
 * @param dist The distribution to sample from.
 * @param dim The dimension of each sample.
 * @param n The number of samples to draw.
 * @param rng The random number generator.
 * @return A vector of Eigen vectors containing the samples, using aligned allocator.
 */
template <typename T>
std::vector<VectorX<T>, Eigen::aligned_allocator<VectorX<T>>>
DrawSamples(Distribution_<T>& distribution, std::size_t n, RandomNumberGenerator& rng) {
  std::vector<VectorX<T>, Eigen::aligned_allocator<VectorX<T>>> samples(n, VectorX<T>::Zero(distribution.DomainSize()));
  distribution.Sample(beluga_estimation::to_view(samples), rng);
  return samples;
}

/**
 * @brief Computes empirical mean and covariance from a collection of samples.
 *
 * @tparam VectorType The Eigen vector type of the samples (deduced from the range).
 * @param samples A range of Eigen vectors.
 * @return A pair of (empirical_mean, empirical_covariance).
 */
template <class Range, typename T = typename ranges::range_value_t<Range>::Scalar>
std::pair<VectorX<T>, MatrixX<T>> ComputeEmpiricalStats(Range&& samples) {
  const std::size_t ndim = samples[0].size();

  VectorX<T> mean = VectorX<T>::Zero(ndim);
  for (const auto& s : samples) {
    mean += s;
  }
  mean /= static_cast<T>(samples.size());

  MatrixX<T> covariance = MatrixX<T>::Zero(ndim, ndim);
  for (const auto& s : samples) {
    const VectorX<T> error = s - mean;
    covariance += error * error.transpose();
  }
  covariance /= static_cast<T>(samples.size() - 1);

  return {mean, covariance};
}

}  // namespace beluga_estimation::testing

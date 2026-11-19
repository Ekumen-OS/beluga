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

#include <memory>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/distributions/se2_tangent_distribution.hpp>

namespace beluga_estimation {

/**
 * @brief Normal Distribution over the $SE(2)$ manifold.
 *
 * Represents a probability distribution where the mean is an $SE(2)$ pose,
 * and the uncertainty is expressed via a $3 \times 3$ covariance matrix
 * defined in the tangent space $\mathfrak{se}(2)$.
 *
 * This is a thin specialization of @ref SE2TangentDistribution_ where the
 * embedded tangent-space distribution is a @ref NormalDistribution_ with
 * zero mean, so the ambient-space mean equals the origin.
 *
 * Marginalization is not supported — @ref Marginalize returns @c nullptr.
 *
 * @tparam T Scalar type (e.g., double, float).
 */
template <typename T>
class SE2NormalDistribution_ : public SE2TangentDistribution_<T> {
 public:
  /**
   * @brief Constructs an SE(2) normal distribution.
   * @param mean The $SE(2)$ mean of the distribution.
   * @param covariance The $3 \times 3$ covariance matrix in the tangent space $\mathfrak{se}(2)$.
   */
  SE2NormalDistribution_(const beluga_estimation::SE2<T>& mean, const beluga_estimation::Matrix3<T>& covariance)
      : SE2TangentDistribution_<T>(std::make_shared<NormalDistribution_<T, 3>>(covariance), mean) {}

  explicit SE2NormalDistribution_(const beluga_estimation::Matrix3<T>& covariance)
      : SE2NormalDistribution_(beluga_estimation::SE2<T>(), covariance) {}

  /**
   * @brief Returns the dimensionality of the distribution domain space.
   * For SE(2), this is always 4 (ambient representation).
   */
  [[nodiscard]] std::size_t DomainSize() const override { return 4; }

  // Bring base class span-based interface into overload set
  using SE2TangentDistribution_<T>::ComputeMean;

  /**
   * @brief Computes the ambient representation of the mean (typed interface).
   *
   * This is the primary implementation that works directly with SE2 types.
   * Uses beluga_estimation::Ref for zero-copy reference semantics with SE2.
   *
   * @param mean A beluga_estimation::Ref to an SE2 object to store the mean `[cos(theta), sin(theta), x, y]`.
   */
  void ComputeMean(beluga_estimation::Ref<beluga_estimation::SE2<T>> mean) const { mean = this->Origin(); }

  /// @brief Returns the SE(2) mean (same as origin).
  [[nodiscard]] const beluga_estimation::SE2<T>& Mean() const { return this->Origin(); }

  /// @brief Returns the 3×3 tangent-space covariance matrix.
  [[nodiscard]] const beluga_estimation::Matrix3<T>& Covariance() const {
    auto embedded = std::static_pointer_cast<const NormalDistribution_<T, 3>>(this->GetEmbeddedDistribution());
    return embedded->Covariance();
  }

  /**
   * @brief Calculates the residual in the tangent space between the distribution's mean and a given state.
   * @param state The $SE(2)$ state to compare against the mean.
   * @return A 3x1 residual vector in the tangent space $\mathfrak{se}(2)$.
   */
  [[nodiscard]] beluga_estimation::Vector3<T> CalculateResidual(const beluga_estimation::SE2<T>& state) const {
    return (this->Mean().inverse() * state).log();
  }
};

using SE2NormalDistribution = SE2NormalDistribution_<DefaultScalar>;

}  // namespace beluga_estimation

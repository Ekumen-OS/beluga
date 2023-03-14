// Copyright 2023 Ekumen, Inc.
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

#ifndef BELUGA_RANDOM_MULTIVARIATE_NORMAL_DISTRIBUTION_HPP
#define BELUGA_RANDOM_MULTIVARIATE_NORMAL_DISTRIBUTION_HPP

#include <random>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <utility>

/**
 * \file
 * \brief Implementation of a multivariate normal distribution.
 */

namespace beluga {

/// Multivariate normal distribution.
/**
 * `MultivariateNormalDistribution<Matrix>` is an implementation of
 * \ref RandomStateDistributionPage that represents a multi-dimensional
 * normal distribution of real-valued random variables.
 *
 * \tparam Matrix The type of the covariance matrix; this is expected to be an instantiation
 * of the [Eigen::Matrix](https://eigen.tuxfamily.org/dox/classEigen_1_1Matrix.html) class
 * template.
 */
template <class Matrix>
class MultivariateNormalDistribution {
 public:
  /// Scalar type for matrices of type Matrix.
  using Scalar = typename Matrix::Scalar;

  /// The eigen column vector from Matrix.
  using Vector = typename Eigen::Vector<typename Matrix::Scalar, Matrix::RowsAtCompileTime>;

  /// Multivariate normal distribution parameter set class.
  class Param {
   public:
    /// The type of distribution to which this parameter set class belongs.
    using distribution_type = MultivariateNormalDistribution<Matrix>;

    /// Constructs a parameter set instance.
    Param() = default;

    /// Constructs a parameter set instance.
    /**
     * \tparam InputType The input type derived from `EigenBase`.
     * \param covariance Real symmetric matrix that represents the covariance of the random variable.
     *
     * \throw std::runtime_error If the provided covariance is invalid.
     */
    template <typename InputType>
    explicit Param(const Eigen::EigenBase<InputType>& covariance) : transform_{make_transform(covariance)} {}

    /// Constructs a parameter set instance.
    /**
     * \tparam InputType The input type derived from `EigenBase`.
     * \param mean A vector that represents the mean value of the random variable.
     * \param covariance Real symmetric matrix that represents the covariance of the random variable.
     *
     * \throw std::runtime_error If the provided covariance is invalid.
     */
    template <typename InputType>
    Param(Vector mean, const Eigen::EigenBase<InputType>& covariance)
        : mean_{std::move(mean)}, transform_{make_transform(covariance)} {}

    /// Compares this object with other parameter set object.
    /**
     * \param other Parameter set object to compare against.
     * \return True if the objects are equal, false otherwise.
     */
    [[nodiscard]] bool operator==(const Param& other) const {
      return mean_ == other.mean_ && transform_ == other.transform_;
    }

    /// Compares this object with other parameter set object.
    /**
     * \param other Parameter set object to compare against.
     * \return True if the objects are not equal, false otherwise.
     */
    [[nodiscard]] bool operator!=(const Param& other) const { return !(*this == other); }

   private:
    friend class MultivariateNormalDistribution<Matrix>;

    Vector mean_{Vector::Zero()};
    Matrix transform_{make_transform(Vector::Ones().asDiagonal())};

    [[nodiscard]] static Matrix make_transform(const Matrix& covariance) {
      // For more information about the method used to generate correlated normal vectors
      // from independent normal distributions, see:
      // https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Drawing_values_from_the_distribution
      // Gentle, J. E. (2009). Computational Statistics. Statistics and Computing. New York: Springer. pp. 315–316.
      // https://doi.org/10.1007%2F978-0-387-98144-4
      if (!covariance.isApprox(covariance.transpose())) {
        throw std::runtime_error("Invalid covariance matrix, it is not symmetric.");
      }
      const auto solver = Eigen::SelfAdjointEigenSolver<Matrix>{covariance};
      if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Invalid covariance matrix, eigen solver failed.");
      }
      const auto& eigenvalues = solver.eigenvalues();
      if ((eigenvalues.array() < 0.0).any()) {
        throw std::runtime_error("Invalid covariance matrix, it has negative eigenvalues.");
      }
      return solver.eigenvectors() * eigenvalues.cwiseSqrt().asDiagonal();
    }
  };

  /// The type of the parameter set.
  using param_type = Param;

  /// The result type generated by the generator.
  using result_type = Vector;

  /// Constructs a MultivariateNormalDistribution with zero mean and circularly symmetric unitary covariance.
  MultivariateNormalDistribution() = default;

  /// Constructs a MultivariateNormalDistribution using param as distribution parameters.
  /**
   * \param params The distribution parameter set.
   */
  explicit MultivariateNormalDistribution(const param_type& params) : params_{params} {}

  /// Constructs a MultivariateNormalDistribution with zero mean and the given covariance.
  /**
   * \tparam InputType The input type derived from `EigenBase`.
   * \param covariance Real symmetric matrix that represents the covariance of the random variable.
   *
   * \throw std::runtime_error If the provided covariance is invalid.
   */
  template <class InputType>
  explicit MultivariateNormalDistribution(const Eigen::EigenBase<InputType>& covariance) : params_{covariance} {}

  /// Constructs a MultivariateNormalDistribution with the given mean and covariance.
  /**
   * \tparam InputType The input type derived from `EigenBase`.
   * \param mean A vector that represents the mean value of the random variable.
   * \param covariance Real symmetric matrix that represents the covariance of the random variable.
   *
   * \throw std::runtime_error If the provided covariance is invalid.
   */
  template <class InputType>
  MultivariateNormalDistribution(const Vector& mean, const Eigen::EigenBase<InputType>& covariance)
      : params_(mean, covariance) {}

  /// Resets the internal state of the distribution.
  void reset() { distribution_.reset(); }

  /// Returns the associated parameter set.
  [[nodiscard]] const param_type& param() const { return params_; }

  /// Sets the associated parameter set to params.
  /**
   * This doesn't reset the internal state of the distribution.
   *
   * \param params New contents of the associated parameter set.
   */
  void param(const param_type& params) { params_ = params; }

  /// Generates the next random object in the distribution.
  /**
   * \tparam Generator The generator type that must meet the requirements of
   * [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator).
   * \param generator An uniform random bit generator object.
   * \return The generated random object.
   */
  template <class Generator>
  [[nodiscard]] result_type operator()(Generator& generator) {
    return this->operator()(generator, params_);
  }

  /// Generates the next random object in the distribution.
  /**
   * \tparam Generator The generator type that must meet the requirements of
   * [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator).
   * \param generator An uniform random bit generator object.
   * \param params Distribution parameter set to use instead of the associated one.
   * \return The generated random object.
   */
  template <class Generator>
  [[nodiscard]] result_type operator()(Generator& generator, const param_type& params) {
    return params.mean_ +
           params.transform_ * Vector{}.unaryExpr([this, &generator](auto) { return distribution_(generator); });
  }

  /// Compares this object with other distribution object.
  /**
   * Two distribution objects are equal when parameter values and internal state are the same.
   * In other words, if they are invoked with equal generators, they generate the same sequence.
   *
   * \param other Distribution object to compare against.
   * \return True if the objects are equal, false otherwise.
   */
  [[nodiscard]] bool operator==(const MultivariateNormalDistribution<Matrix>& other) const {
    return params_ == other.params_ && distribution_ == other.distribution_;
  }

  /// Compares this object with other distribution object.
  /**
   * Two distribution objects are equal when parameter values and internal state are the same.
   * In other words, if they are invoked with equal generators, they generate the same sequence.
   *
   * \param other Distribution object to compare against.
   * \return True if the objects are not equal, false otherwise.
   */
  [[nodiscard]] bool operator!=(const MultivariateNormalDistribution<Matrix>& other) const { return !(*this == other); }

 private:
  param_type params_;
  std::normal_distribution<Scalar> distribution_;
};

/// Deduction guide to help CTAD deduce the correct Eigen type.
template <class InputType>
MultivariateNormalDistribution(const Eigen::EigenBase<InputType>&)
    -> MultivariateNormalDistribution<typename InputType::PlainMatrix>;

/// Deduction guide to help CTAD deduce the correct Eigen type.
template <class InputType, class VectorType>
MultivariateNormalDistribution(const VectorType&, const Eigen::EigenBase<InputType>&)
    -> MultivariateNormalDistribution<typename InputType::PlainMatrix>;

}  // namespace beluga

#endif

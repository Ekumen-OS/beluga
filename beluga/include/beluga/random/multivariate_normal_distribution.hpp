// Copyright 2023-2024 Ekumen, Inc.
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
#include <utility>

#include <beluga/random/multivariate_distribution_traits.hpp>

/**
 * \file
 * \brief Implementation of a multivariate normal distribution.
 */

namespace beluga {

/// Multivariate normal distribution parameter set class.
template <class Vector, class Matrix>
class MultivariateNormalDistributionParam {
 public:
  static_assert(std::is_base_of_v<Eigen::EigenBase<Vector>, Vector>, "Vector should be an Eigen type");
  static_assert(
      Vector::ColsAtCompileTime == 1 || Vector::RowsAtCompileTime == 1,
      "Vector should be a column or row vector");

  /// The scalar type.
  using scalar_type = typename Vector::Scalar;

  /// The vector type.
  using vector_type = Vector;

  /// The covariance matrix from Vector.
  using matrix_type = Matrix;

  /// Constructs a parameter set instance.
  MultivariateNormalDistributionParam() = default;

  /// Constructs a parameter set instance.
  /**
   * \param covariance Real symmetric matrix that represents the covariance of the random variable.
   *
   * \throw std::runtime_error If the provided covariance is invalid.
   */
  explicit MultivariateNormalDistributionParam(matrix_type covariance)
      : transform_{make_transform(std::move(covariance))} {}

  /// Constructs a parameter set instance.
  /**
   * \param mean A vector that represents the mean value of the random variable.
   * \param covariance Real symmetric matrix that represents the covariance of the random variable.
   *
   * \throw std::runtime_error If the provided covariance is invalid.
   */
  MultivariateNormalDistributionParam(vector_type mean, matrix_type covariance)
      : mean_{std::move(mean)}, transform_{make_transform(std::move(covariance))} {}

  /// Compares this object with other parameter set object.
  /**
   * \param other Parameter set object to compare against.
   * \return True if the objects are equal, false otherwise.
   */
  [[nodiscard]] bool operator==(const MultivariateNormalDistributionParam& other) const {
    return mean_ == other.mean_ && transform_ == other.transform_;
  }

  /// Compares this object with other parameter set object.
  /**
   * \param other Parameter set object to compare against.
   * \return True if the objects are not equal, false otherwise.
   */
  [[nodiscard]] bool operator!=(const MultivariateNormalDistributionParam& other) const { return !(*this == other); }

  /// Generates a new random object from the distribution.
  /**
   * \tparam Generator The generator type that must meet the requirements of
   * [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator).
   * \param distribution A reference to a standard normal distribution instance.
   * \param generator An uniform random bit generator object.
   * \return The generated random object.
   */
  template <class Generator>
  [[nodiscard]] auto operator()(std::normal_distribution<scalar_type>& distribution, Generator& generator) const {
    const auto delta = vector_type{}.unaryExpr([&distribution, &generator](auto) { return distribution(generator); });
    if constexpr (vector_type::ColsAtCompileTime == 1) {
      return mean_ + transform_ * delta;
    } else {
      return mean_ + delta * transform_;
    }
  }

 private:
  vector_type mean_{vector_type::Zero()};
  matrix_type transform_{make_transform(vector_type::Ones().asDiagonal())};

  [[nodiscard]] static matrix_type make_transform(matrix_type covariance) {
    // For more information about the method used to generate correlated normal vectors
    // from independent normal distributions, see:
    // https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Drawing_values_from_the_distribution
    // \cite gentle2009computationalstatistics.
    if (!covariance.isApprox(covariance.transpose())) {
      throw std::runtime_error("Invalid covariance matrix, it is not symmetric.");
    }
    const auto solver = Eigen::SelfAdjointEigenSolver<matrix_type>{covariance};
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

/// Multivariate normal distribution.
/**
 * `MultivariateNormalDistribution<T>` is an implementation of
 * \ref RandomStateDistributionPage that represents a multi-dimensional
 * normal distribution of real-valued random variables.
 *
 * \tparam T The result type for the distribution.
 */
template <class T>
class MultivariateNormalDistribution {
 public:
  template <class U>
  friend class MultivariateNormalDistribution;

  /// The scalar type.
  using scalar_type = typename multivariate_distribution_traits<T>::scalar_type;

  /// The Eigen vector type corresponding to T.
  using vector_type = typename multivariate_distribution_traits<T>::vector_type;

  /// The covariance matrix from T.
  using covariance_type = typename multivariate_distribution_traits<T>::covariance_type;

  /// The result type from T.
  using result_type = typename multivariate_distribution_traits<T>::result_type;

  /// The type of the parameter set.
  using param_type = MultivariateNormalDistributionParam<vector_type, covariance_type>;

  /// Construct with zero mean and circularly symmetric unitary covariance.
  MultivariateNormalDistribution() = default;

  /// Construct from distribution parameters.
  /**
   * \param params The distribution parameter set.
   */
  explicit MultivariateNormalDistribution(const param_type& params) : params_{params} {}

  /// Construct with zero mean and the given covariance.
  /**
   * \param covariance Real symmetric matrix that represents the covariance of the random variable.
   *
   * \throw std::runtime_error If the provided covariance is invalid.
   */
  explicit MultivariateNormalDistribution(covariance_type covariance) : params_{std::move(covariance)} {}

  /// Construct with the given mean and covariance.
  /**
   * \param mean An object that represents the mean value of the random variable.
   * \param covariance Real symmetric matrix that represents the covariance of the random variable.
   *
   * \throw std::runtime_error If the provided covariance is invalid.
   */
  MultivariateNormalDistribution(result_type mean, covariance_type covariance)
      : params_{multivariate_distribution_traits<T>::to_vector(std::move(mean)), std::move(covariance)} {}

  /// Copy construct from another compatible distribution.
  /**
   * \tparam U The result type for the other distribution.
   * \param other Another instance of a multivariate normal distribution.
   */
  template <class U>
  /* implicit */ MultivariateNormalDistribution(const MultivariateNormalDistribution<U>& other)  // NOLINT
      : params_{other.params_}, distribution_{other.distribution_} {}

  /// Move construct from another compatible distribution.
  /**
   * \tparam U The result type for the other distribution.
   * \param other Another instance of a multivariate normal distribution.
   */
  template <class U>
  /* implicit */ MultivariateNormalDistribution(MultivariateNormalDistribution<U>&& other) noexcept  // NOLINT
      : params_(std::move(other.params_)), distribution_{std::move(other.distribution_)} {}

  /// Copy assign from another compatible distribution.
  /**
   * \tparam U The result type for the other distribution.
   * \param other Another instance of a multivariate normal distribution.
   */
  template <class U>
  MultivariateNormalDistribution& operator=(const MultivariateNormalDistribution<U>& other) {
    params_ = other.params_;
    distribution_ = other.distribution_;
    return *this;
  }

  /// Move assign from another compatible distribution.
  /**
   * \tparam U The result type for the other distribution.
   * \param other Another instance of a multivariate normal distribution.
   */
  template <class U>
  MultivariateNormalDistribution& operator=(MultivariateNormalDistribution<U>&& other) {
    params_ = std::move(other.params_);
    distribution_ = std::move(other.distribution_);
    return *this;
  }

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
    return (*this)(generator, params_);
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
    return multivariate_distribution_traits<T>::from_vector(params(distribution_, generator));
  }

  /// Compares this object with other distribution object.
  /**
   * Two distribution objects are equal when parameter values and internal state are the same.
   * In other words, if they are invoked with equal generators, they generate the same sequence.
   *
   * \param other Distribution object to compare against.
   * \return True if the objects are equal, false otherwise.
   */
  [[nodiscard]] bool operator==(const MultivariateNormalDistribution<T>& other) const {
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
  [[nodiscard]] bool operator!=(const MultivariateNormalDistribution<T>& other) const { return !(*this == other); }

 private:
  param_type params_;
  std::normal_distribution<scalar_type> distribution_;
};

/// Deduction guide to deduce the correct result type.
template <class T>
MultivariateNormalDistribution(const T&, const typename multivariate_distribution_traits<T>::covariance_type&)
    -> MultivariateNormalDistribution<typename multivariate_distribution_traits<T>::result_type>;

}  // namespace beluga

#endif

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

#ifndef BELUGA_RANDOM_MULTIVARIATE_DISTRIBUTION_TRAITS_HPP
#define BELUGA_RANDOM_MULTIVARIATE_DISTRIBUTION_TRAITS_HPP

#include <Eigen/Core>
#include <sophus/se2.hpp>

/**
 * \file
 * \brief Implementation of multivariate distribution traits.
 */

namespace beluga {

/// Forward declaration of the multivariate_distribution_traits class template.
template <class T, class Enable = void>
struct multivariate_distribution_traits;

/// Specialization for types derived from `Eigen::EigenBase`.
template <class T>
struct multivariate_distribution_traits<T, std::enable_if_t<std::is_base_of_v<Eigen::EigenBase<T>, T>>> {
  static_assert(T::ColsAtCompileTime == 1 || T::RowsAtCompileTime == 1, "T should be a column or row vector");

  /// Extract size information and types from the Eigen matrix type T.
  static constexpr int matrix_size = T::ColsAtCompileTime > T::RowsAtCompileTime  //
                                         ? T::ColsAtCompileTime
                                         : T::RowsAtCompileTime;

  /// The scalar type.
  using scalar_type = typename T::Scalar;

  /// The result type representation.
  using result_type = typename T::PlainMatrix;

  /// The vector type.
  using vector_type = typename T::PlainMatrix;

  /// The covariance matrix type.
  using covariance_type = typename Eigen::Matrix<scalar_type, matrix_size, matrix_size>;

  /// Convert from result to vector representation.
  [[nodiscard]] static constexpr vector_type to_vector(const result_type& t) { return t; }

  /// Convert from vector to result representation.
  [[nodiscard]] static constexpr result_type from_vector(const vector_type& v) { return v; }
};

/// Specialization for types derived from Sophus::SO2Base.
template <class T>
struct multivariate_distribution_traits<T, std::enable_if_t<std::is_base_of_v<Sophus::SO2Base<T>, T>>> {
  /// The scalar type.
  using scalar_type = typename T::Scalar;

  /// The result type representation.
  using result_type = Sophus::SO2<scalar_type>;

  /// The vector type.
  using vector_type = typename Eigen::Matrix<scalar_type, 1, 1>;

  /// The covariance matrix type.
  using covariance_type = typename Eigen::Matrix<scalar_type, 1, 1>;

  /// Convert from result to vector representation.
  [[nodiscard]] static constexpr vector_type to_vector(const result_type& t) { return vector_type{t.log()}; }

  /// Convert from vector to result representation.
  [[nodiscard]] static constexpr result_type from_vector(const vector_type& v) {
    return Sophus::SO2<scalar_type>::exp(v.x());
  }
};

/// Specialization for types derived from Sophus::SE2Base.
template <class T>
struct multivariate_distribution_traits<T, std::enable_if_t<std::is_base_of_v<Sophus::SE2Base<T>, T>>> {
  /// The scalar type.
  using scalar_type = typename T::Scalar;

  /// The result type representation.
  using result_type = Sophus::SE2<scalar_type>;

  /// The vector type.
  using vector_type = typename Eigen::Matrix<scalar_type, 3, 1>;

  /// The covariance matrix type.
  using covariance_type = typename Eigen::Matrix<scalar_type, 3, 3>;

  /// Convert from result to vector representation.
  [[nodiscard]] static constexpr vector_type to_vector(const result_type& t) {
    vector_type v;
    v << t.translation(), t.so2().log();
    return v;
  }

  /// Convert from vector to result representation.
  [[nodiscard]] static constexpr result_type from_vector(const vector_type& v) {
    return {Sophus::SO2<scalar_type>::exp(v.z()), v.head(2)};
  }
};

}  // namespace beluga

#endif

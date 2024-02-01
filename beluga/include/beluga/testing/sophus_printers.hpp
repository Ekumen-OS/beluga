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

#ifndef BELUGA_TESTING_SOPHUS_PRINTERS_HPP
#define BELUGA_TESTING_SOPHUS_PRINTERS_HPP

/**
 * \file
 * \brief Implements printers for Sophus/Eigen types.
 */

#include <iostream>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <sophus/types.hpp>

namespace Eigen {  // NOLINT

/// Printer for Eigen matrices.
/**
 * \tparam Scalar The scalar type.
 * \tparam Rows The number of rows.
 * \tparam Cols The number of columns.
 * \param os An output stream to print to.
 * \param t The value to print.
 */
template <class Scalar, int Rows, int Cols>
std::ostream& operator<<(std::ostream& os, const Eigen::Matrix<Scalar, Rows, Cols>& t) {
  static const auto kSingleLine = Eigen::IOFormat{
      Eigen::StreamPrecision,  //
      Eigen::DontAlignCols,    //
      ", ",                    // coefficient separator
      ", ",                    // row separator
  };
  return os << "(" << t.format(kSingleLine) << ")";
}

}  // namespace Eigen

namespace Sophus {  // NOLINT

/// Printer for SO2 elements.
/**
 * \tparam Scalar The scalar type.
 * \param os An output stream to print to.
 * \param t The value to print.
 */
template <class Scalar>
std::ostream& operator<<(std::ostream& os, const Sophus::SO2<Scalar>& t) {
  return os << t.unit_complex();
}

/// Printer for SE2 elements.
/**
 * \tparam Scalar The scalar type.
 * \param os An output stream to print to.
 * \param t The value to print.
 */
template <class Scalar>
std::ostream& operator<<(std::ostream& os, const Sophus::SE2<Scalar>& t) {
  return os << "(" << t.so2() << ", " << t.translation() << ")";
}

}  // namespace Sophus

#endif

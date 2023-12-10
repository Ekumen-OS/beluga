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

#ifndef BELUGA_AMCL_AMCL_NODE_UTILS_HPP
#define BELUGA_AMCL_AMCL_NODE_UTILS_HPP

#include <utility>
#include <vector>

#include <beluga_ros/messages.hpp>

#include <sophus/se3.hpp>

namespace beluga_amcl::utils {

/// Returns an Eigen format object with comma separators between coefficients.
/**
 * Useful for printing matrices and vectors in a single line.
 */
inline Eigen::IOFormat make_eigen_comma_format() {
  return Eigen::IOFormat{
      Eigen::StreamPrecision, Eigen::DontAlignCols,
      ", ",  // coefficient separator
      ", ",  // row separator
  };
}

}  // namespace beluga_amcl::utils

#endif  // BELUGA_AMCL_AMCL_NODE_UTILS_HPP

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

#include <beluga/type_traits.hpp>
#include <range/v3/view/common.hpp>
#include <sophus/se2.hpp>

namespace beluga {

template <class Derived, class OtherDerived>
inline auto rowwise_covariance(
    const Eigen::MatrixBase<Derived>& matrix,
    const Eigen::MatrixBase<OtherDerived>& rowwise_mean) {
  const auto centered = matrix.colwise() - rowwise_mean;
  return (centered * centered.adjoint()) / (static_cast<typename Derived::Scalar>(matrix.cols() - 1));
}

template <class Mixin>
class SimpleEstimation : public Mixin {
 public:
  template <class... Args>
  explicit SimpleEstimation(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] std::pair<Sophus::SE2d, Eigen::Matrix3d> estimated_pose() const {
    auto&& poses = this->self().states() | ranges::view::common;
    Eigen::Matrix4Xd matrix{4, poses.size()};
    std::transform(poses.begin(), poses.end(), matrix.colwise().begin(), [](const auto& pose) {
      return Eigen::Map<const Eigen::Vector4d>{pose.data()};
    });
    const Sophus::SE2d mean = Eigen::Map<const Sophus::SE2d>{matrix.rowwise().mean().eval().data()};
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    covariance.topLeftCorner<2, 2>() = rowwise_covariance(matrix.bottomRows<2>(), mean.translation());
    covariance.coeffRef(2, 2) = -2.0 * std::log(mean.so2().unit_complex().norm());
    return std::pair{mean, covariance};
  }
};

}  // namespace beluga

#endif

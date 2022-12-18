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

#pragma once

#include <gmock/gmock.h>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace testing {

template <class Scalar>
inline auto Vector2Eq(const Eigen::Vector2<Scalar>& expected) {
  return AllOf(
      Property("x", &Eigen::Vector2<Scalar>::x, DoubleNear(expected.x(), 0.001)),
      Property("y", &Eigen::Vector2<Scalar>::y, DoubleNear(expected.y(), 0.001)));
}

template <class Scalar>
inline auto SO2Eq(const Sophus::SO2<Scalar>& expected) {
  return Property("unit_complex", &Sophus::SO2<Scalar>::unit_complex, Vector2Eq<Scalar>(expected.unit_complex()));
}

template <class Scalar>
inline auto SE2Eq(const Sophus::SE2<Scalar>& expected) {
  return AllOf(
      Property("translation", &Sophus::SE2<Scalar>::translation, Vector2Eq<Scalar>(expected.translation())),
      Property("so2", &Sophus::SE2<Scalar>::so2, SO2Eq<Scalar>(expected.so2())));
}

template <class Scalar>
inline auto SE2Eq(const Sophus::SO2<Scalar>& rotation, const Eigen::Vector2<Scalar>& translation) {
  return SE2Eq<Scalar>(Sophus::SE2<Scalar>{rotation, translation});
}

}  // namespace testing

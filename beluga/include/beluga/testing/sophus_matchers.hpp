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

#ifndef BELUGA_TESTING_SOPHUS_MATCHERS_HPP
#define BELUGA_TESTING_SOPHUS_MATCHERS_HPP

/**
 * \file
 * \brief Implements GTest matchers for Sophus/Eigen types.
 *
 * Make sure you link against the googletest library before including this header.
 * Matchers can be used with EXPECT_THAT and ASSERT_THAT macros.
 */

#include <gmock/gmock.h>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <sophus/types.hpp>

namespace beluga::testing {

using ::testing::AllOf;
using ::testing::DoubleNear;
using ::testing::Property;

/// Vector2 element matcher.
/**
 * \tparam Scalar The scalar type.
 * \param t The value to compare against.
 * \param e The max absolute error.
 */
template <class Scalar>
inline auto Vector2Near(const Sophus::Vector2<Scalar>& t, Scalar e) {
  return AllOf(
      Property("x", &Sophus::Vector2<Scalar>::x, DoubleNear(t.x(), e)),
      Property("y", &Sophus::Vector2<Scalar>::y, DoubleNear(t.y(), e)));
}

/// Vector3 element matcher.
/**
 * \tparam Scalar The scalar type.
 * \param t The value to compare against.
 * \param e The max absolute error.
 */
template <class Scalar>
inline auto Vector3Near(const Sophus::Vector2<Scalar>& t, Scalar e) {
  return AllOf(
      Property("x", &Sophus::Vector2<Scalar>::x, DoubleNear(t.x(), e)),
      Property("y", &Sophus::Vector2<Scalar>::y, DoubleNear(t.y(), e)),
      Property("z", &Sophus::Vector2<Scalar>::z, DoubleNear(t.z(), e)));
}

/// SO2 element matcher.
/**
 * \tparam Scalar The scalar type.
 * \param t The value to compare against.
 * \param e The max absolute error.
 */
template <class Scalar>
inline auto SO2Near(const Sophus::SO2<Scalar>& t, Scalar e) {
  return Property("unit_complex", &Sophus::SO2<Scalar>::unit_complex, Vector2Near(t.unit_complex(), e));
}

/// SE2 element matcher.
/**
 * \tparam Scalar The scalar type.
 * \param t The value to compare against.
 * \param e The max absolute error.
 */
template <class Scalar>
inline auto SE2Near(const Sophus::SE2<Scalar>& t, Scalar e) {
  return AllOf(
      Property("translation", &Sophus::SE2<Scalar>::translation, Vector2Near(t.translation(), e)),
      Property("so2", &Sophus::SE2<Scalar>::so2, SO2Near(t.so2(), e)));
}

/// \overload
template <class Scalar>
inline auto SE2Near(const Sophus::SO2<Scalar>& r, const Sophus::Vector2<Scalar>& t, Scalar e) {
  return SE2Near<Scalar>(Sophus::SE2<Scalar>{r, t}, e);
}

}  // namespace beluga::testing

#endif

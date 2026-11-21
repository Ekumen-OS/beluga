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

#include <optional>

#include <gmock/gmock.h>

#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/core/locally_normal_constraint.hpp>

namespace beluga_estimation {

template <class T>
class MockLocallyNormalConstraint_ : public LocallyNormalConstraint_<T> {
 public:
  using Base = LocallyNormalConstraint_<T>;
  using Base::Base;

  MOCK_METHOD(bool, HasDependentVariables, (), (const, override));
  MOCK_METHOD(bool, HasConditioningVariables, (), (const, override));
  MOCK_METHOD(std_future::span<const Identifier>, DependentVariables, (), (const, override));
  MOCK_METHOD(std_future::span<const Identifier>, ConditioningVariables, (), (const, override));
  MOCK_METHOD(
      T,
      ComputeLogLikelihood,
      (view<std_future::span<const T>>, view<std_future::span<const T>>),
      (const, override));
  MOCK_METHOD(
      void,
      TakeSamples,
      (view<std_future::span<const T>>, view<view<std_future::span<T>>>, RandomNumberGenerator&),
      (const, override));
  MOCK_METHOD(
      void,
      CalcMean,
      (view<std_future::span<const T>>,
       std_future::span<T>,
       std::optional<std_future::mdspan<T, 2>>,
       std::optional<std_future::mdspan<T, 2>>),
      (const, override));
  MOCK_METHOD(void, CalcCovariance, (view<std_future::span<const T>>, std_future::mdspan<T, 2>), (const, override));
  MOCK_METHOD(
      void,
      CalcResidual,
      (view<std_future::span<const T>>,
       view<std_future::span<const T>>,
       std_future::span<T>,
       std::optional<std_future::mdspan<T, 2>>,
       std::optional<std_future::mdspan<T, 2>>),
      (const, override));
  MOCK_METHOD(std::size_t, ResidualSize, (), (const, override));
};

using MockLocallyNormalConstraint = MockLocallyNormalConstraint_<DefaultScalar>;

}  // namespace beluga_estimation

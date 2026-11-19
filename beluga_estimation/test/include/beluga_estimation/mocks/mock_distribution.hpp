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

#include <gmock/gmock.h>

#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/core/distribution.hpp>
#include <beluga_estimation/distributions/dirac_mixture_distribution.hpp>
#include <beluga_estimation/mocks/slice_printers.hpp>

namespace beluga_estimation {

template <typename T>
class MockDistribution_ : public Distribution_<T> {
 public:
  using CovarianceType = std_future::mdspan<T, 2>;

  MOCK_METHOD(std::size_t, DomainSize, (), (const, override));
  MOCK_METHOD(void, ComputeMean, (std_future::span<T> mean), (const, override));
  MOCK_METHOD(void, ComputeCovariance, (CovarianceType covariance), (const, override));
  MOCK_METHOD(void, Sample, (view<std_future::span<T>> out_values, RandomNumberGenerator& rng), (const, override));
  MOCK_METHOD(T, LogProbability, (std_future::span<const T> value), (const, override));
  MOCK_METHOD(
      std::shared_ptr<Distribution_<T>>,
      DoMarginalize,
      (beluga_estimation::view<const contiguous_slice> dims),
      (const, override));
};

using MockDistribution = MockDistribution_<DefaultScalar>;

}  // namespace beluga_estimation

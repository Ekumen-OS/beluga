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

#include <memory>
#include <vector>

#include <gmock/gmock.h>

#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/estimate.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/core/variable.hpp>
#include <beluga_estimation/mocks/mock_distribution.hpp>
#include <beluga_estimation/mocks/mock_manifold.hpp>

namespace beluga_estimation {

template <class T>
class MockEstimate_ : public Estimate_<T> {
 public:
  MOCK_METHOD(
      (beluga_estimation::view<std::shared_ptr<const Variable_<T>>>),
      GetVariables,
      (),
      (const, noexcept, override));
  MOCK_METHOD(std::shared_ptr<const Distribution_<T>>, DoGetDistribution, (), (const, noexcept, override));
  MOCK_METHOD(
      std::shared_ptr<Estimate_<T>>,
      DoMarginalize,
      (std_future::span<const Identifier> variable_ids),
      (const, override));
};

using MockEstimate = MockEstimate_<DefaultScalar>;

}  // namespace beluga_estimation

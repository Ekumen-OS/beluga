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

#include <beluga_estimation/core/variable.hpp>

namespace beluga_estimation {

template <class T>
class MockVariable_ : public Variable_<T> {
 public:
  using Base = Variable_<T>;
  using Base::Base;

  MOCK_METHOD(std::shared_ptr<const beluga_estimation::Manifold_<T>>, Manifold, (), (const, override));
};

using MockVariable = MockVariable_<DefaultScalar>;

}  // namespace beluga_estimation

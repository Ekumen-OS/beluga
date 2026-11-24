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

#include <cstddef>
#include <optional>

#include <gmock/gmock.h>

#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/manifold.hpp>
#include <beluga_estimation/mocks/slice_printers.hpp>

namespace beluga_estimation {

template <class T>
class MockManifold_ : public Manifold_<T> {
 public:
  MOCK_METHOD(std::size_t, AmbientSize, (), (const, override));
  MOCK_METHOD(std::size_t, AmbientSize, (beluga_estimation::view<const contiguous_slice>), (const, override));
  MOCK_METHOD(std::size_t, TangentSize, (), (const, override));
  MOCK_METHOD(
      void,
      Plus,
      (std_future::span<const T>,
       std_future::span<const T>,
       std_future::span<T>,
       (std::optional<std_future::mdspan<T, 2>>),
       (std::optional<std_future::mdspan<T, 2>>)),
      (const, override));
  MOCK_METHOD(
      void,
      Minus,
      (std_future::span<const T>,
       std_future::span<const T>,
       std_future::span<T>,
       (std::optional<std_future::mdspan<T, 2>>),
       (std::optional<std_future::mdspan<T, 2>>)),
      (const, override));
  MOCK_METHOD(
      void,
      LiftAmbient,
      (std_future::span<const T>, beluga_estimation::view<const contiguous_slice>, std_future::span<T>),
      (const, override));
  MOCK_METHOD(
      void,
      EmbedAmbient,
      (std_future::span<const T>, beluga_estimation::view<const contiguous_slice>, std_future::span<T>),
      (const, override));
  MOCK_METHOD(
      std::shared_ptr<const Manifold_<T>>,
      GetSubManifold,
      (beluga_estimation::view<const contiguous_slice>),
      (const, override));
};

using MockManifold = MockManifold_<DefaultScalar>;

}  // namespace beluga_estimation

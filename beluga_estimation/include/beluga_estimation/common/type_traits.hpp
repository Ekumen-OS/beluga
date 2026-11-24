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
#include <optional>

namespace beluga_estimation {

/// @brief Primary template for defining nullable types.
///
/// By default, this maps a type `T` to `std::optional<T>`, making it nullable.
/// Specializations are provided for `std::shared_ptr` and `std::optional` to
/// handle their nullable semantics directly.
///
/// @tparam T The type to make nullable.
template <typename T>
struct nullable {
  using type = std::optional<T>;
};

/// @brief Specialization of `nullable` for `std::shared_ptr<T>`.
///
/// A `std::shared_ptr` is inherently nullable, so its nullable type is itself.
///
/// @tparam T The pointee type of the `std::shared_ptr`.
template <class T>
struct nullable<std::shared_ptr<T>> {
  using type = std::shared_ptr<T>;
};

/// @brief Specialization of `nullable` for `std::optional<T>`.
///
/// An `std::optional` is already a nullable type, so its nullable type is itself.
///
/// @tparam T The value type of the `std::optional`.
template <class T>
struct nullable<std::optional<T>> {
  using type = std::optional<T>;
};

/// @brief Alias for the nullable type of `T`.
///
/// This alias template provides a convenient way to access the underlying nullable
/// type determined by the `nullable` trait.
///
/// @tparam T The type for which to get the nullable equivalent.
template <typename T>
using nullable_t = typename nullable<T>::type;

}  // namespace beluga_estimation

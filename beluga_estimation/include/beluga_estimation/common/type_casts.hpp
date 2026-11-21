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
#include <string_view>
#include <utility>

#include <fmt/format.h>

#include <beluga_estimation/common/result.hpp>

namespace beluga_estimation {

/// @brief Safely performs a `std::dynamic_pointer_cast`, reporting failures as a `Result`.
///
/// If @p pointer is null, returns `Ok(nullptr)`. If the dynamic cast fails,
/// returns an `Error::kFailure` naming @p kind and @p name. Never throws.
///
/// @tparam T The target type to cast to; may be const-qualified.
/// @tparam U The source type to cast from.
/// @param pointer The shared pointer to cast.
/// @param kind Human-readable kind of the element, e.g. `"variable"`.
/// @param name Name of the element, used in the error description.
/// @return `Ok(nullptr)` if @p pointer is null, `Ok(typed)` on success, or an
///         error with `Error::kFailure` if the dynamic type does not match.
template <class T, class U>
Result_<std::shared_ptr<T>>
safe_dynamic_pointer_cast(const std::shared_ptr<U>& pointer, std::string_view kind, std::string_view name) {
  if (pointer == nullptr) {
    return std::shared_ptr<T>{};
  }
  auto typed = std::dynamic_pointer_cast<T>(pointer);
  if (typed == nullptr) {
    return Err(Error::kFailure, fmt::format("{} '{}' has an unexpected type", kind, name));
  }
  return typed;
}

/// @brief Creates a `std::shared_ptr` that shares ownership with another pointer but points to a different object.
///
/// This is a wrapper around the aliasing constructor of `std::shared_ptr`. It is useful for
/// template argument deduction when creating a shared pointer to a member of an owned object.
///
/// @tparam U The type of the original owner.
/// @tparam T The type of the object to point to.
/// @param owner The shared pointer that owns the memory.
/// @param ptr The pointer to the object that the new shared pointer should point to.
/// @return A `std::shared_ptr<T>` that shares ownership with `owner`.
template <typename U, typename T>
std::shared_ptr<T> make_aliased(const std::shared_ptr<U>& owner, T* ptr) {
  return std::shared_ptr<T>(owner, ptr);
}

}  // namespace beluga_estimation

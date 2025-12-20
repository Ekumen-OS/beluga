// Copyright 2025 Ekumen, Inc.
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

#ifndef BELUGA_MIXINS_ENABLE_SELF_HPP
#define BELUGA_MIXINS_ENABLE_SELF_HPP

/**
 * \file
 * \brief Implementation of a CRTP utility to enable downcasting to the most derived class.
 */

namespace beluga {

/// An implementation of a mixin that provides downcasting methods.
/**
 * This class is a mixin that can be used in CRTP base classes to provide convenience
 * downcasting methods to access the most derived class.
 *
 * Typical CRTP example:
 *
 * \code{.cpp}
 * template <typename Derived>
 * struct Base {
 *     void call() {
 *         static_cast<T*>(this)->implementation();
 *     }
 * };
 *
 * struct Derived : public Base<Derived> {
 *     void implementation() {}
 * };
 * \endcode
 *
 * With the `enable_self` mixin:
 *
 * \code{.cpp}
 * template <typename Derived>
 * struct Base : enable_self<Derived> {
 *     using enable_self<Derived>::self;
 *
 *     void call() {
 *         self()->implementation();
 *     }
 * };
 *
 * struct Derived : public Base<Derived> {
 *     void implementation() {}
 * };
 * \endcode
 */
template <typename MostDerived>
struct enable_self {
  /// The type of the most derived class.
  using self_type = MostDerived;

  /// Downcast this to a mutable lvalue reference.
  [[nodiscard]] decltype(auto) self() & { return static_cast<self_type&>(*this); }

  /// Downcast this to a mutable rvalue reference.
  [[nodiscard]] decltype(auto) self() && { return static_cast<self_type&&>(*this); }

  /// Downcast this to a const lvalue reference.
  [[nodiscard]] decltype(auto) self() const& { return static_cast<const self_type&>(*this); }

  /// Downcast this to a const rvalue reference.
  [[nodiscard]] decltype(auto) self() const&& { return static_cast<const self_type&&>(*this); }
};

}  // namespace beluga

#endif  // BELUGA_MIXINS_ENABLE_SELF_HPP

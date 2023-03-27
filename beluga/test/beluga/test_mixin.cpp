// Copyright 2023 Ekumen, Inc.
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

#include <gtest/gtest.h>

#include <beluga/mixin.hpp>

/**
 * \file
 * \brief Tests for mixin utilities and extensions.
 */

namespace {

//! [Using make_mixin]

// Common interface for demonstration purposes.
struct Interface {
  [[nodiscard]] virtual std::string_view name() const = 0;
  virtual ~Interface() = default;
};

// This is a base mixin template that takes two descriptors as parameters.
template <class First, class Second>
using Combined = ciabatta::mixin<First::template mixin, Second::template mixin, ciabatta::provides<Interface>::mixin>;

// This is a mixin component without parameters.
template <class Mixin>
struct EmptyComponent : public Mixin {
  template <class... Args>
  explicit EmptyComponent(Args&&... args) : Mixin(std::forward<Args>(args)...) {}
};

// This is another mixin component without parameters.
template <class Mixin>
struct NameProvider1 : public Mixin {
  template <class... Args>
  explicit NameProvider1(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] std::string_view name() const override { return "One"; }
};

// This is a mixin component with a single parameter.
template <class Mixin>
struct NameProvider2 : public Mixin {
  template <class... Args>
  explicit NameProvider2(char, Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] std::string_view name() const override { return "Two"; }
};

// Descriptor definitions.
using EmptyComponentDescriptor = beluga::mixin::descriptor<EmptyComponent>;
using NameProviderDescriptor1 = beluga::mixin::descriptor<NameProvider1>;
using NameProviderDescriptor2 = beluga::mixin::descriptor<NameProvider2, char>;

TEST(MakeUnique, ExampleBasic) {
  auto first = EmptyComponentDescriptor{};
  auto second = NameProviderDescriptor1{};
  auto ptr = beluga::mixin::make_mixin<Interface, Combined>(first, second);
  static_assert(std::is_same_v<decltype(ptr), std::unique_ptr<Interface>>);
  ASSERT_EQ(ptr->name(), "One");
}

TEST(MakeUnique, ExampleVariants) {
  auto first = EmptyComponentDescriptor{};
  auto second = std::variant<NameProviderDescriptor1, NameProviderDescriptor2>{NameProviderDescriptor1{}};
  auto ptr = beluga::mixin::make_mixin<Interface, Combined>(first, second);
  static_assert(std::is_same_v<decltype(ptr), std::unique_ptr<Interface>>);
  ASSERT_EQ(ptr->name(), "One");
}

TEST(MakeUnique, ExampleParameters) {
  auto first = EmptyComponentDescriptor{};
  auto second = NameProviderDescriptor2{'a'};
  auto ptr = beluga::mixin::make_mixin<Interface, Combined>(first, second);
  static_assert(std::is_same_v<decltype(ptr), std::unique_ptr<Interface>>);
  ASSERT_EQ(ptr->name(), "Two");
}

TEST(MakeUnique, ExampleVariantsAndParameters) {
  auto first = EmptyComponentDescriptor{};
  auto second = std::variant<NameProviderDescriptor1, NameProviderDescriptor2>{NameProviderDescriptor2{'a'}};
  auto ptr = beluga::mixin::make_mixin<Interface, Combined>(first, second);
  static_assert(std::is_same_v<decltype(ptr), std::unique_ptr<Interface>>);
  ASSERT_EQ(ptr->name(), "Two");
}

//! [Using make_mixin]

}  // namespace

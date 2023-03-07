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

#include <gmock/gmock.h>

#include <functional>

#include <beluga/mixin/utility.hpp>

namespace {

struct Object {};

TEST(Filter, FilterWithPredicate) {
  using arithmetic_list = beluga::mixin::filter<std::is_arithmetic, int, float, Object, bool>;
  static_assert(std::is_same_v<arithmetic_list, beluga::mixin::list<int, float, bool>>);
  static_assert(std::is_same_v<beluga::mixin::filter<std::is_void, int, float, Object, bool>, beluga::mixin::list<>>);
}

TEST(IsVariant, Qualified) {
  static_assert(beluga::mixin::is_variant_v<std::variant<int>>);
  static_assert(beluga::mixin::is_variant_v<const std::variant<int>&>);
  static_assert(beluga::mixin::is_variant_v<volatile std::variant<int>>);

  static_assert(!beluga::mixin::is_variant_v<std::variant<int>*>);
  static_assert(!beluga::mixin::is_variant_v<int>);
  static_assert(!beluga::mixin::is_variant_v<Object>);
}

TEST(IsReferenceWrapper, Qualified) {
  static_assert(beluga::mixin::is_reference_wrapper_v<std::reference_wrapper<int>>);
  static_assert(beluga::mixin::is_reference_wrapper_v<const std::reference_wrapper<int>&>);
  static_assert(beluga::mixin::is_reference_wrapper_v<volatile std::reference_wrapper<int>>);

  static_assert(!beluga::mixin::is_reference_wrapper_v<std::reference_wrapper<int>*>);
  static_assert(!beluga::mixin::is_reference_wrapper_v<int>);
  static_assert(!beluga::mixin::is_reference_wrapper_v<Object>);
}

TEST(ForwardLike, Rvalue) {
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<float&>(Object{})), Object&>);
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<const float&>(Object{})), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<float&&>(Object{})), Object&&>);
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<const float&&>(Object{})), const Object&&>);
}

TEST(ForwardLike, MutableLvalue) {
  auto object = Object{};
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<float&>(object)), Object&>);
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<const float&>(object)), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<float&&>(object)), Object&>);
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<const float&&>(object)), const Object&&>);
}

TEST(ForwardLike, ConstLvalue) {
  const auto object = Object{};
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<float&>(object)), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<const float&>(object)), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<float&&>(object)), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::mixin::forward_like<const float&&>(object)), const Object&&>);
}

TEST(ForwardLike, Value) {
  const int value = 200;
  ASSERT_EQ(beluga::mixin::forward_like<Object&>(50), 50);
  ASSERT_EQ(beluga::mixin::forward_like<Object&>(value), 200);
}

TEST(MaybeUnwrap, PassingReferenceWrapper) {
  auto mutable_object = Object{};
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_unwrap(std::ref(mutable_object))), Object&>);
  const auto const_object = Object{};
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_unwrap(std::ref(const_object))), const Object&>);
  int value = 540;
  ASSERT_EQ(beluga::mixin::maybe_unwrap(std::ref(value)), 540);
}

TEST(MaybeUnwrap, PassingNonReferenceWrapper) {
  auto mutable_object = Object{};
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_unwrap(mutable_object)), Object&>);
  const auto const_object = Object{};
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_unwrap(const_object)), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_unwrap(Object{})), Object&&>);
  ASSERT_EQ(beluga::mixin::maybe_unwrap(530), 530);
}

TEST(MaybeVariant, PassingVariant) {
  using variant_type = std::variant<int, float>;
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_variant(variant_type{100})), variant_type&&>);
  auto mutable_object = variant_type{1.0f};
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_variant(mutable_object)), variant_type&>);
  const auto const_object = variant_type{2.0f};
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_variant(const_object)), const variant_type&>);
}

TEST(MaybeVariant, PassingNonVariant) {
  using type = std::variant<std::reference_wrapper<float>>;
  float mutable_object = 2.0f;
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_variant(mutable_object)), type>);
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_variant(std::move(mutable_object))), type>);
  static_assert(std::is_same_v<decltype(beluga::mixin::maybe_variant(100.0f)), type>);
}

TEST(VisitEverything, VariantAndValues) {
  int value = 5;
  auto variant = std::variant<int, long>{2};
  constexpr auto sum_to_long = [](auto v1, auto v2) -> long { return v1 + v2; };
  ASSERT_EQ(beluga::mixin::visit_everything(sum_to_long, variant, value), 7);
  ASSERT_EQ(beluga::mixin::visit_everything(sum_to_long, 10, value), 15);
  ASSERT_EQ(beluga::mixin::visit_everything(sum_to_long, variant, 10), 12);
}

TEST(VisitEverything, FowardSingleParameter) {
  constexpr auto forward = [](auto&& value) -> auto&& {
    return value;
  };
  int value = 5;
  int& alias = beluga::mixin::visit_everything(forward, value);
  ASSERT_EQ(alias, 5);
  alias = 6;
  ASSERT_EQ(value, 6);
}

}  // namespace

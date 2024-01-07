// Copyright 2023-2024 Ekumen, Inc.
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

#include <beluga/utility/forward_like.hpp>

namespace {

struct Object {};

TEST(ForwardLike, Rvalue) {
  static_assert(std::is_same_v<decltype(beluga::forward_like<float&>(Object{})), Object&>);
  static_assert(std::is_same_v<decltype(beluga::forward_like<const float&>(Object{})), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::forward_like<float&&>(Object{})), Object&&>);
  static_assert(std::is_same_v<decltype(beluga::forward_like<const float&&>(Object{})), const Object&&>);
}

TEST(ForwardLike, MutableLvalue) {
  auto object = Object{};
  static_assert(std::is_same_v<decltype(beluga::forward_like<float&>(object)), Object&>);
  static_assert(std::is_same_v<decltype(beluga::forward_like<const float&>(object)), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::forward_like<float&&>(object)), Object&>);
  static_assert(std::is_same_v<decltype(beluga::forward_like<const float&&>(object)), const Object&&>);
}

TEST(ForwardLike, ConstLvalue) {
  const auto object = Object{};
  static_assert(std::is_same_v<decltype(beluga::forward_like<float&>(object)), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::forward_like<const float&>(object)), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::forward_like<float&&>(object)), const Object&>);
  static_assert(std::is_same_v<decltype(beluga::forward_like<const float&&>(object)), const Object&&>);
}

TEST(ForwardLike, Value) {
  const int value = 200;
  ASSERT_EQ(beluga::forward_like<Object&>(50), 50);
  ASSERT_EQ(beluga::forward_like<Object&>(value), 200);
}

}  // namespace

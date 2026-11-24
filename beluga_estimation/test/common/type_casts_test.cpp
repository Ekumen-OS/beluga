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

#include <memory>

#include <gtest/gtest.h>

#include <beluga_estimation/common/type_casts.hpp>

namespace {

struct Base {
  virtual ~Base() = default;
};

struct Derived : Base {
  int value = 42;
};

struct Other {};

TEST(TypeCasts, SafeDynamicPointerCastUpcastSucceeds) {
  auto derived = std::make_shared<Derived>();
  auto result = beluga_estimation::safe_dynamic_pointer_cast<Base>(derived, "object", "derived");
  ASSERT_TRUE(result);
  EXPECT_NE(beluga_estimation::Unwrap(result), nullptr);
}

TEST(TypeCasts, SafeDynamicPointerCastDowncastSucceeds) {
  std::shared_ptr<Base> base = std::make_shared<Derived>();
  auto result = beluga_estimation::safe_dynamic_pointer_cast<Derived>(base, "object", "derived");
  ASSERT_TRUE(result);
  EXPECT_EQ(beluga_estimation::Unwrap(result)->value, 42);
}

TEST(TypeCasts, SafeDynamicPointerCastNullReturnsNull) {
  std::shared_ptr<Base> null_base;
  auto result = beluga_estimation::safe_dynamic_pointer_cast<Derived>(null_base, "object", "derived");
  ASSERT_TRUE(result);
  EXPECT_EQ(beluga_estimation::Unwrap(result), nullptr);
}

TEST(TypeCasts, SafeDynamicPointerCastMismatchReturnsError) {
  std::shared_ptr<Base> base = std::make_shared<Base>();
  auto result = beluga_estimation::safe_dynamic_pointer_cast<Derived>(base, "object", "derived");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kFailure);
  EXPECT_EQ(result.error().GetDescription(), "object 'derived' has an unexpected type");
}

TEST(TypeCasts, SafeDynamicPointerCastMismatchDoesNotThrow) {
  std::shared_ptr<Base> base = std::make_shared<Base>();
  EXPECT_NO_THROW((void)beluga_estimation::safe_dynamic_pointer_cast<Other>(base, "object", "base"));
}

}  // namespace

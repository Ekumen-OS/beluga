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

#include <beluga/mixin/descriptor.hpp>

namespace {

template <class Mixin>
struct MyComponent {};

struct MyComponentParam {};

using Descriptor = beluga::mixin::descriptor<MyComponent>;
using DescriptorWithParams = beluga::mixin::descriptor<MyComponent, MyComponentParam>;

struct StructWithParams {
  int params;
};

struct StructWithParamsMethod {
  int params();
};

template <class... Descriptors>
struct CombinedMixins {};

TEST(Descriptor, IsDescriptor) {
  static_assert(beluga::mixin::is_descriptor_v<Descriptor>);
  static_assert(beluga::mixin::is_descriptor_v<DescriptorWithParams>);
  static_assert(!beluga::mixin::is_descriptor_v<int>);
  static_assert(!beluga::mixin::is_descriptor_v<MyComponentParam>);
}

TEST(Descriptor, HasParams) {
  static_assert(!beluga::mixin::has_params_v<Descriptor>);
  static_assert(beluga::mixin::has_params_v<DescriptorWithParams>);
  static_assert(!beluga::mixin::has_params_v<int>);
  static_assert(!beluga::mixin::has_params_v<MyComponentParam>);
  static_assert(beluga::mixin::has_params_v<StructWithParams>);
  static_assert(!beluga::mixin::has_params_v<StructWithParamsMethod>);
}

TEST(Descriptor, MixinFromDescriptors) {
  using Expected = CombinedMixins<Descriptor, DescriptorWithParams>;
  using FirstMixin = beluga::mixin::mixin_from_descriptors_t<CombinedMixins, Descriptor, DescriptorWithParams>;
  using SecondMixin = beluga::mixin::mixin_from_descriptors_t<CombinedMixins, Descriptor&, const DescriptorWithParams&>;
  using ThirdMixin =
      beluga::mixin::mixin_from_descriptors_t<CombinedMixins, std::tuple<Descriptor&, const DescriptorWithParams&>>;
  static_assert(std::is_same_v<Expected, FirstMixin>);
  static_assert(std::is_same_v<Expected, SecondMixin>);
  static_assert(std::is_same_v<Expected, ThirdMixin>);
}

}  // namespace

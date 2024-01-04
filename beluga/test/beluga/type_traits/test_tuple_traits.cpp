// Copyright 2024 Ekumen, Inc.
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

#include "beluga/type_traits/tuple_traits.hpp"

#include <range/v3/view/subrange.hpp>

namespace {

static_assert(beluga::is_tuple_like_v<std::tuple<int, double>>);
static_assert(beluga::is_tuple_like_v<std::pair<int, float>>);
static_assert(beluga::is_tuple_like_v<std::array<int, 5>>);
static_assert(beluga::is_tuple_like_v<ranges::common_tuple<int, double>>);
static_assert(beluga::is_tuple_like_v<ranges::subrange<ranges::iterator_t<int[3]>, ranges::sentinel_t<int[3]>>>);

static_assert(!beluga::is_tuple_like_v<int>);
static_assert(!beluga::is_tuple_like_v<struct Object>);

static_assert(beluga::tuple_index_v<int, std::tuple<int, double>> == 0);
static_assert(beluga::tuple_index_v<double, std::tuple<int, double>> == 1);

static_assert(beluga::has_single_element_v<int, std::tuple<int, double>>);
static_assert(!beluga::has_single_element_v<int, std::tuple<int, int>>);
static_assert(!beluga::has_single_element_v<int, std::tuple<int, int&>>);

TEST(TupleTraits, ElementAccess) {
  std::pair<int, double> pair{1, 2.0};
  auto& value = beluga::element<int>(pair);
  value = 3;
  ASSERT_EQ(std::get<0>(pair), 3);
}

TEST(TupleTraits, ReferenceElementAccess) {
  double real = 2.0;
  std::tuple<int, double&> pair{1, real};
  auto& value = beluga::element<double>(pair);
  value = 3.0;
  ASSERT_EQ(std::get<1>(pair), 3.0);
  ASSERT_EQ(real, 3.0);
}

}  // namespace

// Copyright 2022 Ekumen, Inc.
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
#include <beluga/type_traits/strongly_typed_numeric.hpp>

namespace beluga {
using strong_double = Numeric<double, struct StrongDoubleTag>;
using strong_size_t = Numeric<std::size_t, struct StrongSizeTTag>;
using strong_size_t_2 = Numeric<std::size_t, struct StrongSizeTTag2>;

TEST(NumericTypeDef, implicitCasts) {
  strong_double sd;
  ASSERT_EQ(sd, 0);
  sd = 2;
  ASSERT_EQ(sd, 2);
  ASSERT_EQ(sd--, 2);
  ASSERT_EQ(sd, 1);
  ASSERT_EQ(sd + 3.4, 4.4);

  // They're not type-safe
  strong_size_t ss = 1;
  strong_size_t_2 ss2 = 1;
  std::size_t r = ss + ss2;
  ASSERT_EQ(r, 2);
}

TEST(NumericTypeDef, numericLimits) {
  ASSERT_EQ(std::numeric_limits<std::size_t>::min(), std::numeric_limits<strong_size_t>::min());
  ASSERT_EQ(std::numeric_limits<double>::min(), std::numeric_limits<strong_double>::min());
}

TEST(NumericTypeDef, hash) {
  ASSERT_EQ(std::hash<std::size_t>{}(12UL), std::hash<strong_size_t>{}(12UL));
  ASSERT_EQ(std::hash<std::size_t>{}(12UL), std::hash<strong_size_t>{}(strong_size_t{12UL}));

  ASSERT_EQ(std::hash<double>{}(12.0), std::hash<strong_double>{}(12.0));
  ASSERT_EQ(std::hash<double>{}(12.0), std::hash<strong_double>{}(strong_double{12.0}));
}
}  // namespace beluga

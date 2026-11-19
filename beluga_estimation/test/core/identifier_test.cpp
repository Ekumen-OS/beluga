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

#include <unordered_set>

#include <gmock/gmock.h>

#include <beluga_estimation/core/identifier.hpp>

namespace {

using beluga_estimation::Identifier;
using beluga_estimation::TimePoint;

TEST(IdentifierTest, DefaultConstructor) {
  const Identifier id;
  EXPECT_EQ(id.name, "");
  EXPECT_EQ(id.timestamp, TimePoint{});
}

TEST(IdentifierTest, Equality) {
  const auto now = std::chrono::system_clock::now();
  const Identifier id1{"a", now};
  const Identifier id2{"a", now};
  const Identifier id3{"b", now};
  const Identifier id4{"a", now + std::chrono::seconds(1)};
  const Identifier id5{"lidar/a", now};

  EXPECT_EQ(id1, id2);
  EXPECT_NE(id1, id3);
  EXPECT_NE(id1, id4);
  EXPECT_NE(id1, id5);
}

TEST(IdentifierTest, Hash) {
  const auto now = std::chrono::system_clock::now();
  const Identifier id1{"a", now};
  const Identifier id2{"a", now};
  const Identifier id3{"b", now};

  std::unordered_set<Identifier> set;
  set.insert(id1);
  set.insert(id2);
  set.insert(id3);

  EXPECT_EQ(set.size(), 2);
  EXPECT_EQ(set.count(id1), 1);
  EXPECT_EQ(set.count(id2), 1);
  EXPECT_EQ(set.count(id3), 1);
}

}  // namespace

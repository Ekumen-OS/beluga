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

#include <array>
#include <iostream>
#include <range/v3/view/span.hpp>

struct Identifier {
  int x;
};
std::array<Identifier, 2> ids;

ranges::span<const Identifier> get_span() {
  return ids;
}

int main() {
  std::cout << get_span().size() << std::endl;
}

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

#pragma once

#include <ostream>

#include <beluga_estimation/common/slice.hpp>

/// @cond INTERNAL
namespace std {

/// @brief Print operator for contiguous_slice, needed by GMock's argument printer.
inline std::ostream& operator<<(std::ostream& os, const beluga_estimation::contiguous_slice& s) {
  os << "contiguous_slice{";
  if (s.start) {
    os << "start=" << *s.start;
  } else {
    os << "start=nullopt";
  }
  os << ", ";
  if (s.end) {
    os << "end=" << *s.end;
  } else {
    os << "end=nullopt";
  }
  os << "}";
  return os;
}

}  // namespace std
/// @endcond

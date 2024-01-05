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

#ifndef BELUGA_DEFINITIONS_HPP
#define BELUGA_DEFINITIONS_HPP

/**
 * \file
 * \brief Convenient library definitions.
 */

/// \cond

#if RANGE_V3_MAJOR == 0 && RANGE_V3_MINOR < 11

// `enable_safe_range` was renamed in range-v3 v0.11.0
// See https://github.com/ericniebler/range-v3/blob/0.12.0/doc/release_notes.md?plain=1#L93
#define enable_borrowed_range enable_safe_range

#endif

/// \endcond

#endif

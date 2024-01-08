// NOLINT(llvm-header-guard)

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

#ifndef BELUGA_PROLOGUE_INCLUDED
#error "Including epilogue, but prologue not included!"
#endif
#undef BELUGA_PROLOGUE_INCLUDED

/// \cond

#if RANGE_V3_MAJOR == 0 && RANGE_V3_MINOR < 11
#undef enable_borrowed_range
#endif

/// \endcond

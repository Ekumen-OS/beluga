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

/**
 * @file
 * @brief Common preprocessor utilities shared across beluga headers.
 */

/// @cond

#define BELUGA_ESTIMATION_STRIP_PARENS(x) BELUGA_ESTIMATION_STRIP_PARENS_IMPL x
#define BELUGA_ESTIMATION_STRIP_PARENS_IMPL(...) __VA_ARGS__

#define BELUGA_ESTIMATION_CONCAT_IMPL(a, b) a##b
#define BELUGA_ESTIMATION_CONCAT(a, b) BELUGA_ESTIMATION_CONCAT_IMPL(a, b)

/// @endcond

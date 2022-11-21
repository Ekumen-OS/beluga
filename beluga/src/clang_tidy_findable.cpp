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

/*
 * This file exists in this header-only library as a workaround for
 * ament_clang_tidy to find and process the headers, since all the source files
 * inside the test folder are ignored.
 * It will not be exported or installed with the rest of the targets of this
 * library.
 */

#include <beluga/beluga.h>

int main() {}

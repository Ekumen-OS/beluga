# Copyright 2025 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@bazel_skylib//lib:modules.bzl", "modules")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _beluga_deps_impl():
    http_archive(
        name = "com_github_hdfgroup_hdf5",
        integrity = "sha256-5N77rDD1DWThVWN0qknldEF8nnLGsd56T/iMSxvqbps=",
        urls = ["https://github.com/HDFGroup/hdf5/releases/download/hdf5_1.14.6/hdf5-1.14.6.tar.gz"],
        strip_prefix = "hdf5-1.14.6",
        build_file = "//deps/com_github_hdfgroup_hdf5:package.BUILD.bazel",
    )

beluga_deps = modules.as_extension(_beluga_deps_impl)

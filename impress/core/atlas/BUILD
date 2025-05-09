# Copyright 2024 Google LLC
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

load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "atlas_packer",
    srcs = [
        "atlas_packer.cc",
    ],
    hdrs = [
        "atlas_packer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/math",
        "//core/view/utils:macros",
        "@com_google_absl//absl/types:optional",
    ],
    alwayslink = 1,
)

cc_library(
    name = "shelf_atlas_packer",
    srcs = [
        "shelf_atlas_packer.cc",
    ],
    hdrs = [
        "shelf_atlas_packer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":atlas_packer",
        "//core/common:robin_map",
        "//core/math",
    ],
    alwayslink = 1,
)

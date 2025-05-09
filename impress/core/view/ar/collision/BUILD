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

licenses(["notice"])

cc_library(
    name = "collision",
    srcs = [
        "ar_collision_system.cc",
        "collider_ar_plane.cc",
    ],
    hdrs = [
        "ar_collision_system.h",
        "collider_ar_magical_surface_point.h",
        "collider_ar_plane.h",
        "collider_ar_point.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//core/ar",
        "//core/ar:shared",
        "//core/collision:collider_helpers",
        "//core/collision:ray",
        "//core/common",
        "//core/common:log",
        "//core/common:robin_map",
        "//core/ncsb",
        "//core/view/ar:base_ar_component",
        "//core/view/framework",
        "@com_google_absl//absl/types:optional",
    ],
    alwayslink = 1,
)

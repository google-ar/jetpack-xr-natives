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

load("//build_tools:imp.bzl", "imp_defines", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "materials",
    srcs = [
        "material.cc",
    ],
    hdrs = [
        "material.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/common:log",
        "//core/common:owned_or_borrowed_ptr",
        "//core/common:owned_or_unowned_memory",
        "//core/common:owned_ptr",
        "//core/material_library:material_param_value",
        "//core/render",
        "//core/split_engine:split_engine_serializer",
        "//core/view:base_view",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

exports_files([
    "color_conversion_helpers.glsl",
    "super_sample.glsl",
])

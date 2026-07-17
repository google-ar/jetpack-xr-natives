# Copyright 2025 Google LLC
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

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "if_dev_runtime", "imp_defines", "imp_google3_copts")
load(
    "//core/proto:imp_cc_proto_library.bzl",
    "imp_cc_proto_library",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

proto_library(
    name = "camera_state_proto",
    srcs = ["camera_state.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "camera_proto_cc_imp",
    proto = ":camera_state_proto",
)

cc_library(
    name = "camera",
    srcs = [
        "camera_component.cc",
        "camera_manager.cc",
    ],
    hdrs = [
        "camera_component.h",
        "camera_manager.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":camera_proto_cc_imp",
        "//core:config",
        "//core/collision:collision_helpers",
        "//core/collision:plane",
        "//core/collision:ray",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view:view_events",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ] + if_dev_runtime([
        "//core/editor/widgets/viewport:viewport_helpers",
    ]),
    alwayslink = 1,
)

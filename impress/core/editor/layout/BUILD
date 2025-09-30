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

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "imp_defines", "imp_google3_copts")
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

cc_library(
    name = "editor_panel_ids",
    hdrs = [
        "editor_panel_ids.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    alwayslink = 1,
)

cc_library(
    name = "layout",
    srcs = [
        "layout_composer.cc",
    ],
    hdrs = [
        "layout_composer.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":docking_manager",
        ":editor_panel_ids",
        ":helpers",
        ":layout_config_imp_cc_proto",
        "//core:config",
        "//core/common:invocable",
        "//core/editor:editor_constants",
        "//core/editor:widget",
        "//core/editor:widget_layout_info",
        "//core/math",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "docking_manager",
    srcs = [
        "docking_manager.cc",
    ],
    hdrs = [
        "docking_manager.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":editor_panel_ids",
        "@com_google_absl//absl/strings:string_view",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "helpers",
    hdrs = [
        "helpers.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":editor_control_flags",
        "//core/common:bit_flag",
        "//core/math",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

proto_library(
    name = "layout_config_proto",
    srcs = ["layout_config.proto"],
    deps = [
        "//core/math:math_proto",
    ],
)

imp_cc_proto_library(
    name = "layout_config_imp_cc_proto",
    proto = ":layout_config_proto",
    deps = [
        "//core/math:math_proto_cc",
    ],
)

cc_library(
    name = "editor_control_flags",
    hdrs = [
        "editor_control_flags.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/common:bit_flag",
    ],
    alwayslink = 1,
)

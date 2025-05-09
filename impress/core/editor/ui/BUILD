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

# Common UI abstractions for the editor.
load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "ui_helpers",
    srcs = [
        "ui_helpers.cc",
    ],
    hdrs = [
        "ui_helpers.h",
    ],
    copts = imp_google3_copts(),
    deps = ["@dear_imgui"],
    alwayslink = 1,
)

cc_library(
    name = "directory_ui",
    hdrs = [
        "directory_ui.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "directory_ui_desktop",
    srcs = [
        "directory_ui_desktop.cc",
    ],
    hdrs = [
        "directory_ui_desktop.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":directory_ui",
        ":ui_helpers",
        "//core/assets",
        "//core/async:future",
        "//core/common",
        "//core/common:log",
        "//core/editor/widgets/icons:texture_assets",
        "//core/render",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "directory_ui_noop",
    srcs = [
        "directory_ui_noop.cc",
    ],
    hdrs = [
        "directory_ui_noop.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":directory_ui",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "drag_and_drop",
    srcs = [
        "drag_and_drop.cc",
    ],
    hdrs = [
        "drag_and_drop.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:type_traits",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

# Note: this is broken out of drag_and_drop to avoid a dependency cycle:
# core/ncsb
# core/editor:editor_field_control
# core/editor/ui:drag_and_drop
# core/ncsb <--
cc_library(
    name = "drag_and_drop_node",
    srcs = [
        "drag_and_drop_node.cc",
    ],
    hdrs = [
        "drag_and_drop_node.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":drag_and_drop",
        "//core/common:type_traits",
        "//core/ncsb",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "filterable_combo",
    srcs = [
        "filterable_combo.cc",
    ],
    hdrs = [
        "filterable_combo.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:invocable",
        "//core/editor:widget",
        "//core/editor/layout:editor_control_flags",
        "//core/editor/layout:helpers",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "euler_angle_field",
    srcs = [
        "euler_angle_field.cc",
    ],
    hdrs = [
        "euler_angle_field.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/editor/layout:editor_control_flags",
        "//core/editor/layout:helpers",
        "//core/math",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

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
load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "editor",
    srcs = [
        "recipe_editor.cc",
    ],
    hdrs = [
        "recipe_editor.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":constants",
        ":editor_graph",
        ":helpers",
        ":simple_connection_node_placements",
        "//core/async:future",
        "//core/common:rememberer",
        "//core/common:robin_map",
        "//core/editor",
        "//core/editor:editor_field_control",
        "//core/editor:events",
        "//core/editor:widget",
        "//core/math",
        "//core/ncsb",
        "//core/recipes:recipe_runner",
        "//core/recipes/language:recipe_graph_cc",
        "//core/recipes/language:recipe_runtime_graph",
        "//core/recipes/language:recipe_scope",
        "//core/recipes/language:recipe_types_cc",
        "//core/recipes/language:recipe_utils",
        "//core/view:base_view",
        "//core/view/utils:string_map",
        "//third_party/imgui_node_editor",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "editor_graph",
    srcs = [
        "recipe_editor_graph.cc",
    ],
    hdrs = [
        "recipe_editor_graph.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":constants",
        "//core/common:robin_map",
        "//core/recipes/language:recipe_graph_cc",
        "//core/recipes/language:recipe_utils",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "icons",
    srcs = [
        "icons.cc",
    ],
    hdrs = [
        "icons.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "helpers",
    srcs = [
        "editor_helpers.cc",
    ],
    hdrs = [
        "editor_helpers.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":icons",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "constants",
    hdrs = [
        "editor_constants.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "node_placements",
    hdrs = [
        "node_placements.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":editor_graph",
        "//core/common:robin_map",
        "//core/math",
    ],
    alwayslink = 1,
)

cc_library(
    name = "linear_node_placements",
    srcs = [
        "linear_node_placements.cc",
    ],
    hdrs = [
        "linear_node_placements.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":editor_graph",
        ":node_placements",
        "//core/common:robin_map",
        "//core/math",
        "//third_party/imgui_node_editor",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "simple_connection_node_placements",
    srcs = [
        "simple_connection_node_placements.cc",
    ],
    hdrs = [
        "simple_connection_node_placements.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":editor_graph",
        ":node_placements",
        "//core/common:robin_map",
        "//core/math",
        "//third_party/imgui_node_editor",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

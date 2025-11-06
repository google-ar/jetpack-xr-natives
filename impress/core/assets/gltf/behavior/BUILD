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

licenses(["notice"])

cc_library(
    name = "converted_graph",
    srcs = [
        "converted_graph.cc",
    ],
    hdrs = [
        "converted_graph.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:platform_helpers",
        "//core/model",
        "//core/recipes/language:recipe_graph_cc",
        "//core/recipes/language:recipe_utils",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "node_converters",
    srcs = [
        "node_converters.cc",
    ],
    hdrs = [
        "node_converters.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":converted_graph",
        ":node_converter_constants",
        ":world_pointer",
        "//core/math",
        "//core/model",
        "//core/recipes/language:recipe_graph_cc",
        "//core/recipes/language:recipe_utils",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:variant",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "world_pointer",
    srcs = [
        "world_pointer.cc",
    ],
    hdrs = [
        "world_pointer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "node_converter_constants",
    hdrs = [
        "node_converter_constants.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

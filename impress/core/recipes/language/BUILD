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

load("//build_tools:imp.bzl", "imp_google3_copts", "imp_test_copts")
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
    name = "recipe_graph",
    srcs = ["recipe_graph.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/ncsb:node_handle_message_proto",
        "//core/proto:imp_proto",
        "//core/scene_handles:scene_handles_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

imp_cc_proto_library(
    name = "recipe_graph_cc",
    proto = ":recipe_graph",
    deps = [
        "//core/ncsb:node_handle_message_imp_proto",
        "//core/proto:any_imp_proto",
        "//core/scene_handles:scene_handles_imp_cc_proto",
    ],
)

cc_library(
    name = "recipe_traits",
    hdrs = [
        "recipe_traits.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_utils",
    ],
    alwayslink = 1,
)

cc_library(
    name = "recipe_utils",
    srcs = [
        "recipe_utils.cc",
    ],
    hdrs = [
        "recipe_utils.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_graph_cc",
        "//core/async:future",
        "//core/common:invocable",
        "//core/common:log",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/proto:any_imp_proto",
        "//core/scene_handles",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:variant",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "recipe_runtime_event",
    hdrs = [
        "recipe_runtime_event.h",
    ],
    copts = imp_google3_copts(),
    deps = [":recipe_utils"],
    alwayslink = 1,
)

cc_library(
    name = "base_recipe_system",
    hdrs = [
        "base_recipe_system.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_graph_cc",
        ":recipe_traits",
        ":recipe_utils",
        ":registered_function",
        "//core/async:future",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "recipe_system",
    srcs = [
        "recipe_system.cc",
    ],
    hdrs = [
        "recipe_system.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":base_recipe_system",
        ":recipe_graph_cc",
        ":recipe_scope",
        ":recipe_utils",
        ":registered_function",
        "//core/assets/gltf:gltf_audio_extension",
        "//core/async:future",
        "//core/common:filament_helpers",
        "//core/common:log",
        "//core/common:registry",
        "//core/common:type_traits",
        "//core/math",
        "//core/ncsb",
        "//core/proto:any_imp_proto",
        "//core/recipes/language/functions/math",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:string_map",
        "//proposed:delayed_future_scheduler",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "recipe_scope",
    srcs = [
        "recipe_scope.cc",
    ],
    hdrs = [
        "recipe_scope.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_graph_cc",
        ":recipe_utils",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
    alwayslink = 1,
)

cc_library(
    name = "recipe_runtime_graph",
    srcs = [
        "recipe_runtime_graph.cc",
    ],
    hdrs = [
        "recipe_runtime_graph.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_graph_cc",
        ":recipe_runtime_event",
        ":recipe_scope",
        ":recipe_system",
        ":recipe_traits",
        ":recipe_utils",
        "//core/async:future",
        "//core/common:invocable",
        "//core/common:registry",
        "//core/common:type_traits",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:variant",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "registered_function",
    srcs = ["registered_function.cc"],
    hdrs = ["registered_function.h"],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_utils",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
    alwayslink = 1,
)

cc_library(
    name = "function_test_helpers",
    testonly = 1,
    hdrs = ["function_test_helpers.h"],
    copts = imp_test_copts(),
    deps = [
        ":recipe_utils",
        "//:testing_api",
        "@com_google_absl//absl/status:statusor",
        "@mediapipe//mediapipe/framework/port:gtest_main",
    ],
    alwayslink = 1,
)

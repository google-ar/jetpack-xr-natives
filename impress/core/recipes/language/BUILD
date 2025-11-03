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
        ":recipe_types",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "recipe_graph_cc",
    proto = ":recipe_graph",
    deps = [
        ":recipe_types_cc",
    ],
)

proto_library(
    name = "recipe_types",
    srcs = ["recipe_types.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/ncsb:node_handle_message_proto",
        "//core/proto:imp_proto",
        "//core/scene_handles:scene_handles_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

imp_cc_proto_library(
    name = "recipe_types_cc",
    proto = ":recipe_types",
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
        "//core/math",
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
        ":recipe_types_cc",
        "//core/async:future",
        "//core/common:invocable",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/proto:any_imp_proto",
        "//core/scene_handles",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
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
        ":recipe_custom_statement",
        ":recipe_traits",
        ":recipe_types_cc",
        ":recipe_utils",
        ":registered_function",
        "//core/async:future",
        "//core/common:invocable",
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
        ":recipe_custom_statement",
        ":recipe_scope",
        ":recipe_types_cc",
        ":recipe_utils",
        ":registered_function",
        "//core/assets/gltf:gltf_audio_extension",
        "//core/async:future",
        "//core/common:filament_helpers",
        "//core/common:invocable",
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
        "@com_google_impress//core/common:log",
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
        ":recipe_types_cc",
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
        ":binary_expression",
        ":recipe_async_execution_manager",
        ":recipe_custom_statement",
        ":recipe_execution_context",
        ":recipe_graph_cc",
        ":recipe_runtime_event",
        ":recipe_scope",
        ":recipe_system",
        ":recipe_types_cc",
        ":recipe_utils",
        ":unary_expression",
        "//core/async:future",
        "//core/common:invocable",
        "//core/common:registry",
        "//core/common:type_traits",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
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
        "//core/recipes/tests:recipe_matchers",
        "@com_google_absl//absl/status:statusor",
        "@mediapipe//mediapipe/framework/port:gtest_main",
    ],
    alwayslink = 1,
)

cc_library(
    name = "recipe_custom_statement",
    hdrs = ["recipe_custom_statement.h"],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_async_execution_manager",
        ":recipe_execution_context",
        ":recipe_graph_cc",
        ":recipe_utils",
        "//core/async:future",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "recipe_async_execution_manager",
    srcs = [
        "recipe_async_execution_manager.cc",
    ],
    hdrs = [
        "recipe_async_execution_manager.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_graph_cc",
        ":recipe_scope",
        ":recipe_utils",
        "//core/common:ref_counter",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "recipe_execution_context",
    hdrs = ["recipe_execution_context.h"],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_async_execution_manager",
        ":recipe_scope",
        "//core/view:base_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "binary_expression",
    srcs = ["binary_expression.cc"],
    hdrs = ["binary_expression.h"],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_graph_cc",
        ":recipe_traits",
        ":recipe_utils",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
    alwayslink = 1,
)

cc_library(
    name = "unary_expression",
    srcs = ["unary_expression.cc"],
    hdrs = ["unary_expression.h"],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_graph_cc",
        ":recipe_traits",
        ":recipe_utils",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:variant",
    ],
    alwayslink = 1,
)

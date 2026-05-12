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
load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "if_dev_runtime", "imp_google3_copts")
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

cc_library(
    name = "recipe_event",
    srcs = [
        "recipe_event.cc",
    ],
    hdrs = [
        "recipe_event.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/ncsb",
        "//core/recipes/language:recipe_utils",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

proto_library(
    name = "recipe_runner_proto",
    srcs = ["recipe_runner_state.proto"],
    deps = [
        "//core/recipes/language:recipe_graph",
    ],
)

imp_cc_proto_library(
    name = "recipe_runner_proto_cc",
    proto = ":recipe_runner_proto",
    deps = ["//core/recipes/language:recipe_graph_cc"],
)

config_setting(
    name = "imp_enable_recipe_experimental",
    define_values = {
        "IMP_ENABLE_RECIPE_EXPERIMENTAL": "1",
    },
)

cc_library(
    name = "recipe_runner",
    srcs = [
        "recipe_runner.cc",
    ],
    hdrs = [
        "recipe_runner.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":recipe_event",
        ":recipe_runner_proto_cc",
        "//core:config",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/input",
        "//core/math",
        "//core/ncsb",
        "//core/recipes/language:recipe_async_execution_manager",
        "//core/recipes/language:recipe_execution_context",
        "//core/recipes/language:recipe_graph_cc",
        "//core/recipes/language:recipe_runtime_event",
        "//core/recipes/language:recipe_runtime_graph",
        "//core/recipes/language:recipe_scope",
        "//core/recipes/language:recipe_system",
        "//core/recipes/language:recipe_types_cc",
        "//core/recipes/language:recipe_utils",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//split_engine/input:split_engine_input_event",
    ] + if_dev_runtime([
        "//core/editor:editor_info",
    ]),
    alwayslink = 1,
)

imp_assets(
    name = "test_assets",
    srcs = [
        "//core/recipes/data:test_simple_graph_isf",
    ],
    namespace = "imp::test::recipe",
)

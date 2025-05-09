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

load("//build_tools:imp.bzl", "imp_test_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/*.png"]),
)

cc_library(
    name = "test_helpers",
    testonly = 1,
    hdrs = [
        "test_helpers.h",
    ],
    copts = imp_test_copts(),
    deps = [
        "//core/assets/gltf/behavior:converted_graph",
        "//core/assets/gltf/behavior:node_converters",
        "//core/assets/gltf/behavior:world_pointer",
        "//core/common:imp_matchers",
        "//core/common:registry",
        "//core/math",
        "//core/model",
        "//core/recipes/language:recipe_graph_cc",
        "//core/recipes/language:recipe_runtime_event",
        "//core/recipes/language:recipe_runtime_graph",
        "//core/recipes/language:recipe_scope",
        "//core/recipes/language:recipe_system",
        "//core/recipes/language:recipe_utils",
        "//core/view:base_view",
        "@mediapipe//mediapipe/framework/port:gtest_main",
    ],
    alwayslink = 1,
)

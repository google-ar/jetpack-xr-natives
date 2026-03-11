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
load(
    "//build_tools:imp.bzl",
    "imp_test_copts",
)

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "api_bindings_test",
    testonly = True,
    hdrs = ["api_bindings_test.h"],
    copts = imp_test_copts(),
    linkstatic = True,
    deps = [
        ":mock_asset_loader",
        "//apibindings:asset_ptr_map",
        "//apibindings:base_asset_loader",
        "//apibindings:generic_material_manager_interface",
        "//apibindings:impress_api_view",  # buildcleaner: keep
        "//apibindings:impress_api_view_header",
        "//apibindings:model_manager_interface",
        "//apibindings:skybox_manager_interface",
        "//apibindings:stereo_surface_manager_interface",
        "//apibindings:texture_manager_interface",
        "//apibindings:water_material_manager_interface",
        "//core/common:invocable",
        "//core/view/framework/assets/tests:test_resources",
        "//core/view/utils:asset",
        "//core/window:default_lighting",
        "//testing",
        "@mediapipe//mediapipe/framework/port:gtest",
    ],
    alwayslink = 1,
)

cc_library(
    name = "mock_asset_animator",
    testonly = True,
    hdrs = ["mock_asset_animator.h"],
    deps = [
        "//apibindings:base_asset_animator",
        "@mediapipe//mediapipe/framework/port:gtest_main",
    ],
    alwayslink = 1,
)

cc_library(
    name = "mock_asset_loader",
    testonly = True,
    hdrs = ["mock_asset_loader.h"],
    deps = [
        "//apibindings:base_asset_loader",
        "@mediapipe//mediapipe/framework/port:gtest_main",
    ],
    alwayslink = 1,
)

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
load("//build_tools:imp_material.bzl", "imp_material")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "xr_editor_ui",
    srcs = [
        "xr_editor_ui.cc",
    ],
    hdrs = ["xr_editor_ui.h"],
    deps = [
        "//core/editor:editor_constants",
        "//core/editor/components:world_space_editor_ui",
        "//core/editor/layout:editor_panel_ids",
        "//core/math",
        "//core/ncsb",
        "//core/view/utils:string_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "xr_grab_handle",
    srcs = [
        "xr_grab_handle.cc",
    ],
    hdrs = ["xr_grab_handle.h"],
    deps = [
        ":xr_grab_handle_assets",
        "//core/actions:action_config",
        "//core/actions:controller_events",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/camera",
        "//core/collision:ray",
        "//core/common:registry",
        "//core/editor",
        "//core/input",
        "//core/materials",
        "//core/math",
        "//core/model/mesh:mesh_factory",
        "//core/ncsb",
        "//core/render:mesh_renderer",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "//core/xr:openxr_events",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//split_engine/input:split_engine_input_event",
    ],
    alwayslink = 1,
)

imp_material(
    name = "solid_material",
    src = ":solid.mat",
)

imp_assets(
    name = "xr_grab_handle_assets",
    srcs = [
        ":solid_material",
    ],
    namespace = "editor::xr_grab_handle_assets",
)

# Copyright 2026 Google LLC
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

# This folder contains code for the Magic Window related components. The
# goal is to separate Magic Window from the generic Gsplat components in
# core/gsplat/. There should be no dependencies from core/gsplat/ into here.

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//core/proto:imp_cc_proto_library.bzl", "imp_cc_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "magic_window_renderer_state_proto",
    srcs = ["magic_window_renderer_state.proto"],
    deps = [
        "//core/gsplat:gsplat_proto",
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "magic_window_renderer_state_proto_cc",
    proto = ":magic_window_renderer_state_proto",
    deps = [
        "//core/gsplat:gsplat_cc_proto",
        "//core/ncsb:node_handle_message_imp_proto",
    ],
)

cc_library(
    name = "magic_window_renderer",
    srcs = ["magic_window_renderer.cc"],
    hdrs = ["magic_window_renderer.h"],
    deps = [
        ":magic_window_renderer_state_proto_cc",
        "//:api",
        "//core/assets",
        "//core/async:future",
        "//core/common:owned_ptr",
        "//core/gsplat:gsplat_asset",
        "//core/math",
        "//core/ncsb",
        "//core/render",
        "//core/render_passes:texture_pipeline_renderer_projection_quad",
        "//core/split_engine:fbs",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@third_party//split_engine/materials:gsplat_material_serializer",
    ],
    alwayslink = 1,
)

cc_library(
    name = "loading_quad",
    srcs = ["loading_quad.cc"],
    hdrs = ["loading_quad.h"],
    deps = [
        "//:api",
        "//core/input",
        "//core/material_library:fbs",
        "//core/material_library:generic_material_spec",
        "//core/material_library:material_param_value",
        "//core/math",
        "//core/model/mesh:mesh_factory",
        "//core/ncsb",
        "//core/render:mesh_renderer",
        "//core/split_engine/materials:split_engine_generic_material",
        "//core/view/utils:frame_time",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

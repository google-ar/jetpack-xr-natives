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

load("@rules_cc//cc:cc_library.bzl", "cc_library")
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
    name = "group_to_projection_quad_texture_renderer_state_proto",
    srcs = ["group_to_projection_quad_texture_renderer_state.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_editor_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "group_to_projection_quad_texture_renderer_state_proto_cc_imp",
    proto = "group_to_projection_quad_texture_renderer_state_proto",
    deps = [
        "//core/math:math_proto_cc",
        "//core/proto:imp_cc_proto",
    ],
)

cc_library(
    name = "group_to_projection_quad_texture_renderer_component",
    srcs = ["group_to_projection_quad_texture_renderer.cc"],
    hdrs = ["group_to_projection_quad_texture_renderer.h"],
    deps = [
        ":group_to_projection_quad_texture_renderer_state_proto_cc_imp",
        "//core/async:future",
        "//core/common:optional_with_default",
        "//core/math",
        "//core/ncsb",
        "//core/render_passes:texture_pipeline_renderer",
        "//core/render_passes:texture_pipeline_renderer_helper",
        "//core/render_passes:texture_pipeline_renderer_projection_quad",
        "//core/render_passes:texture_pipeline_renderer_proto_cc_imp",
        "//core/view/utils:frame_time",
        "//core/view/utils:render_settings_proto_cc_imp",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

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
load("//build_tools:imp.bzl", "imp_copts")
load("//build_tools:imp_material.bzl", "imp_material")
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
    name = "surface_renderer",
    srcs = [
        "surface_renderer.cc",
    ],
    hdrs = [
        "surface_renderer.h",
    ],
    copts = imp_copts(),
    deps = [
        ":surface_renderer_proto_cc_imp",
        "//:api",
        "//core/common:log",
        "//core/view:view_events",
        "//core/view/framework",
        "//core/view/utils:render_setting_utils",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

proto_library(
    name = "surface_renderer_scripting_proto",
    srcs = [
        "proto/surface_renderer_scripting.proto",
    ],
    deps = [
        "//core/math:math_proto",
        "//core/ncsb:node_handle_message_proto",
        "//core/proto:imp_proto",
        "//core/view/utils:render_settings_proto",
    ],
)

imp_cc_proto_library(
    name = "surface_renderer_proto_cc_imp",
    proto = ":surface_renderer_scripting_proto",
    deps = [
        "//core/math:math_proto_cc",
        "//core/ncsb:node_handle_message_imp_proto",
        "//core/view/utils:render_settings_proto_cc_imp",
    ],
)

java_lite_proto_library(
    name = "surface_renderer_scripting_java_proto_lite",
    deps = [":surface_renderer_scripting_proto"],
)

proto_library(
    name = "texture_pipeline_renderer_proto",
    srcs = [
        "texture_pipeline_renderer_state.proto",
    ],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
        "//core/scene_handles:scene_handles_proto",
        "//core/view/framework:framework_proto",
        "//core/view/utils:render_settings_proto",
    ],
)

imp_cc_proto_library(
    name = "texture_pipeline_renderer_proto_cc_imp",
    proto = ":texture_pipeline_renderer_proto",
    deps = [
        ":texture_config_proto_cc_imp",
        "//core/math:math_proto_cc",
        "//core/scene_handles:scene_handles_imp_cc_proto",
        "//core/view/utils:render_settings_proto_cc_imp",
    ],
)

cc_library(
    name = "texture_pipeline_renderer",
    srcs = [
        "texture_pipeline_renderer.cc",
    ],
    hdrs = [
        "texture_pipeline_renderer.h",
    ],
    copts = imp_copts(),
    deps = [
        ":texture_pipeline_renderer_proto_cc_imp",
        "//core/async:future",
        "//core/common:log",
        "//core/common:small_source_location",
        "//core/materials",
        "//core/math",
        "//core/ncsb",
        "//core/render",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/framework",
        "//core/view/utils:device",
        "//core/view/utils:render_setting_utils",
        "//core/view/utils:render_settings_proto_cc_imp",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)

proto_library(
    name = "on_demand_texture_pipeline_render_params_proto",
    srcs = [
        "on_demand_texture_pipeline_render_params.proto",
    ],
    deps = [
        ":texture_config_proto",
        "//core/math:math_proto",
        "//core/proto:imp_proto",
        "//core/scene_handles:scene_handles_proto",
        "//core/view/utils:render_settings_proto",
    ],
)

imp_cc_proto_library(
    name = "on_demand_texture_pipeline_render_params_proto_cc_imp",
    proto = ":on_demand_texture_pipeline_render_params_proto",
    deps = [
        ":texture_config_proto_cc_imp",
        "//core/math:math_proto_cc",
        "//core/scene_handles:scene_handles_imp_cc_proto",
        "//core/view/utils:render_settings_proto_cc_imp",
    ],
)

proto_library(
    name = "texture_config_proto",
    srcs = [
        "texture_config.proto",
    ],
)

imp_cc_proto_library(
    name = "texture_config_proto_cc_imp",
    proto = ":texture_config_proto",
)

cc_library(
    name = "on_demand_texture_pipeline_renderer",
    srcs = [
        "on_demand_texture_pipeline_renderer.cc",
    ],
    hdrs = ["on_demand_texture_pipeline_renderer.h"],
    copts = imp_copts(),
    deps = [
        ":on_demand_texture_pipeline_render_params_proto_cc_imp",
        ":texture_config_proto_cc_imp",
        ":texture_config_utils",
        "//core/common:log",
        "//core/math",
        "//core/render",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:render_setting_utils",
        "//core/view/utils:render_settings_proto_cc_imp",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "texture_config_utils",
    srcs = ["texture_config_utils.cc"],
    hdrs = ["texture_config_utils.h"],
    deps = [
        ":texture_config_proto_cc_imp",
        "//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

imp_material(
    name = "sin_distortion",
    src = "sin_distortion.mat",
)

imp_material(
    name = "normals_as_color",
    src = "normals_as_color.mat",
)

imp_material(
    name = "color",
    src = "color.mat",
)

imp_material(
    name = "texture",
    src = "texture.mat",
)

imp_assets(
    name = "texture_pipeline_renderer_test_assets",
    srcs = [
        ":color",
        ":normals_as_color",
        ":sin_distortion",
        ":texture",
        "//third_party/arcore/googledata/sceneform/assets/gltf_2p0_sample_assets:DamagedHelmet_glb",
    ],
    namespace = "imp::texture_pipeline_renderer_test_assets",
)

cc_library(
    name = "message_handler_lib",
    srcs = [
        "message_handlers/create_node_with_surface_renderer_handler.cc",
        "message_handlers/surface_renderer_message_handler.cc",
    ],
    hdrs = [
        "message_handlers/create_node_with_surface_renderer_handler.h",
        "message_handlers/surface_renderer_message_handler.h",
    ],
    deps = [
        ":surface_renderer",
        ":surface_renderer_proto_cc_imp",
        "//core/common:log",
        "//core/common:platform_helpers",
        "//core/scripting:message_handler",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
    ],
    alwayslink = 1,
)

# --config=android_all
# --android_ndk_min_sdk_version=23

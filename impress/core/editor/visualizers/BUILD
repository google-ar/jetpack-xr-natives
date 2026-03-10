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

# IMGUI UI Widgets for Impress Previewer.
load("//build_tools:imp.bzl", "imp_google3_copts")
load("//core/proto:imp_isf_data.bzl", "imp_isf_data")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "visualizer",
    srcs = [
        "camera_visualizer.cc",
        "light_visualizer.cc",
        "visualizer_manager.cc",
    ],
    hdrs = [
        "camera_visualizer.h",
        "light_visualizer.h",
        "visualizer.h",
        "visualizer_manager.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":camera_visualizer_assets",
        "//core/camera",
        "//core/common:invocable",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/editor",
        "//core/editor/visualizers/light_assets:light_visualizer_assets",
        "//core/lighting:light_component",
        "//core/lighting:light_state_proto_cc_imp",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

imp_assets(
    name = "camera_visualizer_assets",
    srcs = [
        ":camera_isf",
        "//core/editor/visualizers:camera_visualizer.gltf",
    ],
    namespace = "imp::editor::camera_visualizer_assets",
)

imp_isf_data(
    name = "camera_isf",
    srcs = [
        "camera_visualizer.textproto",
    ],
)

cc_library(
    name = "material_visualizer",
    srcs = [
        "material_visualizer.cc",
    ],
    hdrs = [
        "material_visualizer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":material_visualizer_assets",
        "//core/assets",
        "//core/async:future",
        "//core/common:hash",
        "//core/common:registry",
        "//core/editor",
        "//core/math",
        "//core/ncsb",
        "//core/proto:proto_writer",
        "//core/render",
        "//core/render:primitive_shape_renderer",
        "//core/render:primitive_shape_renderer_state_cc_proto",
        "//core/render_passes:on_demand_texture_pipeline_render_params_proto_cc_imp",
        "//core/render_passes:on_demand_texture_pipeline_renderer",
        "//core/render_passes:texture_config_proto_cc_imp",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:render_settings_proto_cc_imp",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

imp_assets(
    name = "material_visualizer_assets",
    srcs = [
        ":material_visualizer_scene",
    ],
    namespace = "imp::editor",
)

imp_isf_data(
    name = "material_visualizer_scene",
    srcs = [
        "material_visualizer.textproto",
    ],
    deps = [
        "//core/render:primitive_shape_renderer_state_proto",
        "//core/render_passes:texture_pipeline_renderer_proto",
    ],
)

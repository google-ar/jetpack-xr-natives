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

# Components for Impress Previewer.
load("//build_tools:imp.bzl", "if_android", "imp_google3_copts")
load("//build_tools:imp_material.bzl", "imp_material")
load("//core/proto:imp_cc_proto_library.bzl", "imp_cc_proto_library")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "camera_rotate",
    srcs = [
        "camera_rotate.cc",
    ],
    hdrs = [
        "camera_rotate.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/actions:input_action_event",
        "//core/common:registry",
        "//core/editor",
        "//core/editor:editor_plugin",
        "//core/editor:events",
        "//core/input",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "camera_translate",
    srcs = [
        "camera_translate.cc",
    ],
    hdrs = [
        "camera_translate.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/camera",
        "//core/collision:ray",
        "//core/common:registry",
        "//core/editor",
        "//core/editor:editor_plugin",
        "//core/input",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "camera_zoom",
    srcs = [
        "camera_zoom.cc",
    ],
    hdrs = [
        "camera_zoom.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/common:registry",
        "//core/editor",
        "//core/editor:editor_plugin",
        "//core/editor:events",
        "//core/input",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "temporary_editor_metadata",
    hdrs = [
        "temporary_editor_metadata.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":temporary_editor_metadata_state_proto_cc",
        "//core/ncsb",
    ],
    alwayslink = 1,
)

proto_library(
    name = "temporary_editor_metadata_state_proto",
    srcs = ["temporary_editor_metadata_state.proto"],
)

imp_cc_proto_library(
    name = "temporary_editor_metadata_state_proto_cc",
    proto = ":temporary_editor_metadata_state_proto",
)

imp_assets(
    name = "grid_asset",
    srcs = [
        ":grid_material",
    ],
    namespace = "imp::editor::grid_data",
)

imp_material(
    name = "grid_material",
    src = ":grid.mat",
)

cc_library(
    name = "grid",
    srcs = [
        "grid.cc",
    ],
    hdrs = [
        "grid.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":grid_asset",
        "//core/collision:collision_helpers",
        "//core/collision:plane",
        "//core/collision:ray",
        "//core/common:registry",
        "//core/editor",
        "//core/editor:events",
        "//core/math",
        "//core/ncsb",
        "//core/render:primitive_shape_renderer",
        "//core/render:primitive_shape_renderer_state_cc_proto",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "@com_google_absl//absl/status",
    ],
    alwayslink = 1,
)

imp_assets(
    name = "test_assets",
    srcs = [
        "//third_party/arcore/googledata/sceneform/assets/gltf_2p0_sample_assets:BoxAnimated_glb",
    ],
    namespace = "imp::editor::test_assets",
)

cc_library(
    name = "world_space_editor_ui",
    srcs = [
        "spatial_ui_canvas.cc",
        "world_space_editor_ui.cc",
    ],
    hdrs = [
        "spatial_ui_canvas.h",
        "world_space_editor_ui.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":world_space_editor_ui_assets",
        "//core:config",
        "//core/actions:action_config",
        "//core/actions:controller_events",
        "//core/assets",
        "//core/async:future",
        "//core/common:debug_draw",
        "//core/common:registry",
        "//core/editor",
        "//core/editor/xr:xr_grab_handle",
        "//core/input",
        "//core/materials",
        "//core/math",
        "//core/model/mesh",
        "//core/ncsb",
        "//core/render",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:frame_time",
        "//core/view/utils:string_map",
        "//core/window:filament_host",
        "//core/window:imgui_renderer",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine/input:split_engine_input_event",
        "@third_party//split_engine/materials:texture_external_material",
    ] + if_android(
        [
            "//core/window:open_gl_imgui_renderer",
            "//core/render/android:android_external_texture_surface",
        ],
    ),
    alwayslink = 1,
)

imp_material(
    name = "world_space_editor_ui_texture",
    src = ":world_space_editor_ui_texture.mat",
)

imp_material(
    name = "spatial_ui_canvas_texture",
    src = ":spatial_ui_canvas_texture.mat",
)

imp_material(
    name = "spatial_ui_canvas_ogl_texture",
    src = ":spatial_ui_canvas_ogl_texture.mat",
)

imp_assets(
    name = "world_space_editor_ui_assets",
    srcs = [
        ":spatial_ui_canvas_ogl_texture",
        ":spatial_ui_canvas_texture",
        ":world_space_editor_ui_texture",
    ],
    namespace = "imp::editor::world_space_editor_ui_assets",
)

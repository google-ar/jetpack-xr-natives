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

# IMGUI UI Widgets for Impress Editor.
load("//build_tools:imp.bzl", "imp_google3_copts", "imp_linkopts", "imp_test_copts")
load("//core/proto:imp_cc_proto_library.bzl", "imp_cc_proto_library")
load("//core/proto:imp_isf_data.bzl", "imp_isf_data")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "asset_type_helpers",
    hdrs = [
        "asset_type_helpers.h",
    ],
    copts = imp_google3_copts(),
    deps = ["@com_google_absl//absl/strings"],
    alwayslink = 1,
)

cc_library(
    name = "asset_thumbnail_provider",
    srcs = [
        "asset_thumbnail_provider.cc",
    ],
    hdrs = [
        "asset_thumbnail_provider.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":asset_type_helpers",
        "//core/assets",
        "//core/async:future",
        "//core/common",
        "//core/editor/visualizers:material_visualizer",
        "//core/editor/widgets/icons:texture_assets",
        "//core/render",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:asset",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "asset_library",
    srcs = [
        "asset_library.cc",
    ],
    hdrs = [
        "asset_library.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":asset_thumbnail_provider",
        ":asset_type_helpers",
        "//core:config",
        "//core/async:future",
        "//core/common",
        "//core/common:invocable",
        "//core/common:rememberer",
        "//core/editor:widget",
        "//core/editor/layout:editor_control_flags",
        "//core/editor/layout:helpers",
        "//core/editor/ui:directory_ui",
        "//core/editor/ui:directory_ui_desktop",
        "//core/editor/ui:directory_ui_noop",
        "//core/editor/ui:drag_and_drop",
        "//core/editor/ui:drag_and_drop_node",
        "//core/editor/ui:ui_helpers",
        "//core/editor/widgets/icons:texture_assets",
        "//core/ncsb:ncsb_proto_cc",
        "//core/proto:proto_writer",
        "//core/proto:textproto_writer",
        "//core/render",
        "//core/resources",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:string_map",
        "//core/view/utils:string_set",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "component_ui",
    srcs = [
        "component_ui.cc",
    ],
    hdrs = [
        "component_ui.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":component_widgets",
        "//core:config",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/common:robin_map",
        "//core/editor",
        "//core/editor:editor_info",
        "//core/editor:events",
        "//core/editor:widget",
        "//core/editor:widget_layout_info",
        "//core/editor:widget_ui_system",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "console",
    srcs = [
        "console.cc",
    ],
    hdrs = [
        "console.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/assets",
        "//core/async:future",
        "//core/common:platform_helpers",
        "//core/common:rememberer",
        "//core/editor:editor_style",
        "//core/editor:widget",
        "//core/editor/widgets/icons:texture_assets",
        "//core/render",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@dear_imgui",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "event_injector",
    srcs = [
        "event_injector.cc",
    ],
    hdrs = [
        "event_injector.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/editor:editor_field_control",
        "//core/editor:widget",
        "//core/editor/ui:filterable_combo",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "file_drag_and_drop",
    srcs = [
        "file_drag_and_drop.cc",
    ],
    hdrs = [
        "file_drag_and_drop.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/editor:editor_utils",
        "//core/editor:file_loader_helper",
        "//core/editor:file_type_loader",
        "//core/editor:file_type_registry",
        "//core/editor:widget",
        "//core/editor/ui:drag_and_drop",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/framework",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "hierarchy",
    srcs = [
        "hierarchy.cc",
    ],
    hdrs = [
        "hierarchy.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/common:robin_map",
        "//core/editor",
        "//core/editor:editor_clipboard",
        "//core/editor:editor_info",
        "//core/editor:editor_style",
        "//core/editor:editor_touch",
        "//core/editor:events",
        "//core/editor:widget",
        "//core/editor/layout:editor_control_flags",
        "//core/editor/layout:helpers",
        "//core/editor/ui:drag_and_drop_node",
        "//core/input",
        "//core/math",
        "//core/ncsb",
        "//core/ncsb:node_flag",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "settings_widget",
    srcs = [
        "settings_widget.cc",
    ],
    hdrs = [
        "settings_widget.h",
        "settings_widget_constants.h",
    ],
    copts = imp_test_copts(),
    deps = [
        ":asset_library",
        "//core:config",
        "//core/common:registry",
        "//core/editor",
        "//core/editor:events",
        "//core/editor:widget",
        "//core/ncsb",
        "//core/view:base_view",
        "@com_google_absl//absl/strings:string_view",
        "@dear_imgui",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "materials_widget",
    srcs = [
        "materials_widget.cc",
    ],
    hdrs = [
        "materials_widget.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/editor",
        "//core/editor:events",
        "//core/editor:widget",
        "//core/materials",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "node_details",
    srcs = [
        "node_details.cc",
    ],
    hdrs = [
        "node_details.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":asset_library",
        "//core:config",
        "//core/common",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/editor",
        "//core/editor:command",
        "//core/editor:editor_constants",
        "//core/editor:editor_field_control",
        "//core/editor:editor_style",
        "//core/editor:events",
        "//core/editor:events_cc_proto",
        "//core/editor:node_value_command",
        "//core/editor:widget",
        "//core/editor/layout:editor_control_flags",
        "//core/ncsb",
        "//core/ncsb:ncsb_proto_cc",
        "//core/proto:proto_writer",
        "//core/proto:textproto_writer",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "toggle_camera",
    srcs = ["toggle_camera.cc"],
    hdrs = ["toggle_camera.h"],
    deps = [
        "//core/assets",
        "//core/async:future",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/editor",
        "//core/editor:events",
        "//core/editor:widget",
        "//core/editor/widgets/icons:texture_assets",
        "//core/ncsb",
        "//core/render",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "transform",
    srcs = [
        "transform.cc",
        "transform_widget.cc",
    ],
    hdrs = [
        "transform.h",
        "transform_widget.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":transform_assets",
        ":transform_widget_aspect",
        ":transform_widget_mode_control",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/editor",
        "//core/editor:command",
        "//core/editor:editor_field_control",
        "//core/editor:editor_style",
        "//core/editor:events",
        "//core/editor:events_cc_proto",
        "//core/editor:node_value_command",
        "//core/editor:widget",
        "//core/editor/ui:euler_angle_field",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "transform_widget_aspect",
    srcs = [
        "transform_widget_aspect.cc",
    ],
    hdrs = [
        "transform_widget_aspect.h",
    ],
    copts = imp_google3_copts(),
    linkopts = imp_linkopts(),
    deps = [
        ":transform_widget_aspect_state_imp_cc_proto",
        "//core/collision:collision_helpers",
        "//core/collision:ray",
        "//core/common:registry",
        "//core/editor",
        "//core/editor:command",
        "//core/editor:events",
        "//core/editor:events_cc_proto",
        "//core/editor:node_value_command",
        "//core/math",
        "//core/ncsb",
        "//core/scene_handles:scene_handle_status_utils",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

proto_library(
    name = "transform_widget_aspect_state_proto",
    srcs = ["transform_widget_aspect_state.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/scene_handles:scene_handles_proto",
    ],
)

imp_cc_proto_library(
    name = "transform_widget_aspect_state_imp_cc_proto",
    proto = ":transform_widget_aspect_state_proto",
    deps = [
        ":transform_widget_mode_control",
        "//core/math:math_proto_cc",
    ],
)

cc_library(
    name = "transform_widget_mode_control",
    srcs = [
        "transform_widget_mode_control.cc",
    ],
    hdrs = [
        "transform_widget_mode_control.h",
    ],
    copts = imp_google3_copts(),
    linkopts = imp_linkopts(),
    deps = [
        ":transform_widget_mode_control_state_imp_cc_proto",
        "//core/common:registry",
        "//core/editor",
        "//core/editor:events",
        "//core/ncsb",
        "//core/view/framework",
    ],
    alwayslink = 1,
)

cc_library(
    name = "component_widgets",
    srcs = [
        "fallback_component_widget.cc",
    ],
    hdrs = [
        "component_editor_helpers.h",
        "fallback_component_widget.h",
        "isf_component_widget.h",
        "stateless_isf_component_widget.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:invocable",
        "//core/common:registry",
        "//core/editor:command",
        "//core/editor:component_editor_cc_proto",
        "//core/editor:editor_field_control",
        "//core/editor:editor_info",
        "//core/editor:editor_style",
        "//core/editor:events_cc_proto",
        "//core/editor:node_value_command",
        "//core/editor:widget",
        "//core/editor/layout:editor_control_flags",
        "//core/editor/layout:helpers",
        "//core/ncsb",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

proto_library(
    name = "transform_widget_mode_control_state_proto",
    srcs = ["transform_widget_mode_control_state.proto"],
    deps = ["//core/scene_handles:scene_handles_proto"],
)

imp_cc_proto_library(
    name = "transform_widget_mode_control_state_imp_cc_proto",
    proto = ":transform_widget_mode_control_state_proto",
    deps = [
        "//core/math:math_proto_cc",
        "//core/scene_handles:scene_handles_imp_cc_proto",
    ],
)

imp_assets(
    name = "transform_assets",
    srcs = [
        ":transform_isf",
        ":transform_widget.glb",
    ],
    namespace = "imp::editor::transform_assets",
)

imp_isf_data(
    name = "transform_isf",
    srcs = [
        "transform_widget.textproto",
    ],
    deps = [
        ":transform_widget_aspect_state_proto",
        ":transform_widget_mode_control_state_proto",
    ],
)

cc_library(
    name = "visualize_bounds",
    srcs = [
        "visualize_bounds.cc",
    ],
    hdrs = [
        "visualize_bounds.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:debug_draw",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/editor",
        "//core/editor:events",
        "//core/editor:widget",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/ncsb:node_flag",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

imp_assets(
    name = "visualization_test_assets",
    srcs = [
        "visualize_test_asset_spheres.glb",
        "visualize_test_asset_toy_car.glb",
    ],
    namespace = "imp::editor::visualization_test_assets",
)

cc_library(
    name = "visualize_colliders",
    srcs = [
        "visualize_colliders.cc",
    ],
    hdrs = [
        "visualize_colliders.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/collision:collision_flags",
        "//core/common:debug_draw",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/editor",
        "//core/editor:events",
        "//core/editor:widget",
        "//core/math",
        "//core/ncsb",
        "//core/ncsb:node_flag",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "vertex_select_widget",
    srcs = [
        "vertex_select_widget.cc",
    ],
    hdrs = [
        "vertex_select_widget.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/collision:collision_helpers",
        "//core/common:debug_draw",
        "//core/common:rememberer",
        "//core/editor:widget",
        "//core/math",
        "//core/model",
        "//core/model:model_data_details",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "visualize_origins",
    srcs = [
        "visualize_origins.cc",
    ],
    hdrs = [
        "visualize_origins.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:debug_draw",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/editor",
        "//core/editor:events",
        "//core/editor:widget",
        "//core/math",
        "//core/ncsb",
        "//core/ncsb:node_flag",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "environment_light_editor",
    srcs = [
        "environment_light_editor.cc",
    ],
    hdrs = [
        "environment_light_editor.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/editor:widget",
        "//core/editor/layout:editor_control_flags",
        "//core/editor/layout:helpers",
        "//core/editor/ui:euler_angle_field",
        "//core/lighting:environment_light",
        "//core/lighting:image_based_lighting_asset",
        "//core/math",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@dear_imgui",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "editor_mode_toggle",
    srcs = ["editor_mode_toggle.cc"],
    hdrs = ["editor_mode_toggle.h"],
    deps = [
        "//core/assets",
        "//core/async:future",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/editor",
        "//core/editor:editor_info",
        "//core/editor:widget",
        "//core/editor/widgets/icons:texture_assets",
        "//core/render",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "widget_scuba_view_fixture",
    testonly = True,
    hdrs = ["widget_scuba_view_fixture.h"],
    copts = imp_test_copts(),
    deps = [
        "//core/editor:editor_constants",
        "//core/editor:widget_ui_system",
        "//core/editor/layout",
        "//core/editor/layout:layout_config_imp_cc_proto",
        "//core/window:filament_host",
        "//testing:imgui_scuba_view_fixture",
    ],
    alwayslink = 1,
)

cc_library(
    name = "filament_view_settings",
    srcs = ["filament_view_settings.cc"],
    hdrs = ["filament_view_settings.h"],
    deps = [
        "//core/math",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "filament_view_settings_widget",
    srcs = ["filament_view_settings_widget.cc"],
    hdrs = ["filament_view_settings_widget.h"],
    deps = [
        ":filament_view_settings",
        "//core/editor:widget",
        "//core/math",
        "//core/view:base_view",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings:string_view",
        "@dear_imgui",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:filagui",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

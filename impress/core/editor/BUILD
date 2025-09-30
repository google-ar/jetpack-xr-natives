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

# Impress Editor.
load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "if_desktop", "if_emscripten", "imp_defines", "imp_google3_copts")
load("//core/proto:imp_cc_proto_library.bzl", "imp_cc_proto_library")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "editor_info",
    srcs = ["editor_info.cc"],
    hdrs = ["editor_info.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/common:registry",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "editor",
    hdrs = ["editor.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":editor_info",
        ":editor_plugin",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_absl//absl/base:core_headers",
    ],
    alwayslink = 1,
)

#TODO Establish better separation of editor/editor_impl so we don't need `# buildcleaner: keep`.
cc_library(
    name = "editor_impl",
    srcs = [
        "editor_impl.cc",
        "selection_controller_impl.cc",
    ],
    hdrs = ["selection_controller_impl.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":command",
        ":editor",
        ":editor_constants",
        ":editor_field_control",
        ":editor_info",
        ":editor_input_handler",
        ":editor_plugin",
        ":editor_style",
        ":events",
        ":file_loader_helper",
        ":file_type_registry",
        ":selection_controller",
        ":widget",
        ":widget_layout_info",
        ":widget_ui_system",
        "//core:config",
        "//core/assets/gltf:gltf_audio_extension",
        "//core/assets/gltf:gltf_behavior_extension",
        "//core/assets/gltf:gltf_interactivity_extension",
        "//core/async:future",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/editor/components:camera_rotate",
        "//core/editor/components:camera_translate",
        "//core/editor/components:camera_zoom",
        "//core/editor/components:grid",
        "//core/editor/layout",
        "//core/editor/layout:editor_control_flags",
        "//core/editor/layout:editor_panel_ids",
        "//core/editor/layout:layout_config_imp_cc_proto",
        "//core/editor/visualizers:visualizer",
        "//core/editor/widgets:asset_library",
        "//core/editor/widgets:component_ui",
        "//core/editor/widgets:console",
        "//core/editor/widgets:editor_mode_toggle",
        "//core/editor/widgets:environment_light_editor",
        "//core/editor/widgets:event_injector",
        "//core/editor/widgets:filament_view_settings_widget",
        "//core/editor/widgets:file_drag_and_drop",
        "//core/editor/widgets:hierarchy",
        "//core/editor/widgets:materials_widget",
        "//core/editor/widgets:node_details",
        "//core/editor/widgets:settings_widget",
        "//core/editor/widgets:toggle_camera",
        "//core/editor/widgets:transform",
        "//core/editor/widgets:vertex_select_widget",
        "//core/editor/widgets:visualize_bounds",
        "//core/editor/widgets:visualize_colliders",
        "//core/editor/widgets:visualize_origins",
        "//core/editor/widgets/performance",
        "//core/input",
        "//core/math",
        "//core/ncsb",
        "//core/ncsb:ncsb_proto_cc",
        "//core/ncsb:node_flag",
        "//core/scene_handles",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/platforms/android/wrappers:imp_lifecycle_callback",
        "//previewer/widgets:header_widgets",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_impress//core/common:log",
    ] + if_desktop([
    ]) + if_emscripten([
    ]),
    alwayslink = 1,
)

cc_library(
    name = "editor_utils",
    srcs = [
        "editor_utils.cc",
    ],
    hdrs = [
        "editor_utils.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":editor",
        "//core/collision:collision_helpers",
        "//core/math",
        "//core/view:base_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "editor_style",
    srcs = [
        "editor_style.cc",
    ],
    hdrs = [
        "editor_style.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "selection_controller",
    hdrs = ["selection_controller.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/ncsb",
    ],
    alwayslink = 1,
)

filegroup(
    name = "Box_glb",
    srcs = ["gltf/Box.glb"],
)

imp_assets(
    name = "editor_impl_test_assets",
    srcs = [
        ":Box_glb",
    ],
    namespace = "imp::editor::test_assets",
)

cc_library(
    name = "events",
    hdrs = [
        "events.h",
    ],
    copts = imp_google3_copts(),
    deps = ["//core/ncsb"],
    alwayslink = 1,
)

proto_library(
    name = "events_proto",
    srcs = ["events.proto"],
    deps = [
        ":component_editor_proto",
        "//core/math:math_proto",
        "//core/ncsb:node_handle_message_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "events_cc_proto",
    proto = ":events_proto",
    deps = [
        ":component_editor_cc_proto",
        "//core/ncsb:node_handle_message_imp_proto",
        "//core/proto:any_imp_proto",
    ],
)

cc_library(
    name = "widget",
    hdrs = [
        "widget.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/common:invocable",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "widget_layout_info",
    hdrs = [
        "widget_layout_info.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = ["//core/editor/layout:editor_panel_ids"],
    alwayslink = 1,
)

cc_library(
    name = "widget_ui_system",
    srcs = [
        "widget_ui_system.cc",
    ],
    hdrs = [
        "widget_ui_system.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":widget",
        ":widget_layout_info",
        "//core/common:hash",
        "//core/common:type_traits",
        "//core/editor/components:world_space_editor_ui",
        "//core/editor/layout",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view:view_events",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "editor_field_control",
    hdrs = [
        "editor_field_control.h",
        "editor_proto_visitor.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":editor_style",
        "//core/common:bit_flag",
        "//core/common:platform_helpers",
        "//core/common:template_helpers",
        "//core/editor/layout:editor_control_flags",
        "//core/editor/layout:helpers",
        "//core/editor/ui:drag_and_drop",
        "//core/editor/ui:drag_and_drop_node",
        "//core/editor/widgets:asset_type_helpers",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/proto:imp_editor_imp_cc_proto",
        "//core/proto:proto_common",
        "//core/proto:proto_differ",
        "//core/proto:proto_reader",
        "//core/proto:proto_writer",
        "//core/resources",
        "//core/scene_handles:scene_handle_interface",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "editor_constants",
    hdrs = [
        "editor_constants.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/editor/layout:layout_config_imp_cc_proto",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "editor_input_handler",
    srcs = ["editor_input_handler.cc"],
    hdrs = ["editor_input_handler.h"],
    deps = [
        ":editor",
        ":editor_constants",
        "//core/common:registry",
        "//core/input",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_absl//absl/algorithm:container",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "file_type",
    srcs = ["file_type.cc"],
    hdrs = ["file_type.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "file_type_loader",
    srcs = [],
    hdrs = ["file_type_loader.h"],
    copts = imp_google3_copts(),
    deps = [
        "//core/async:future",
        "//core/common:invocable",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "file_type_registry",
    srcs = ["file_type_registry.cc"],
    hdrs = ["file_type_registry.h"],
    copts = imp_google3_copts(),
    deps = [
        ":file_type",
        ":file_type_loader",
        "//core/async:future",
        "//core/common:invocable",
        "//core/ncsb",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
    alwayslink = 1,
)

cc_library(
    name = "file_loader_helper",
    srcs = ["file_loader_helper.cc"],
    hdrs = ["file_loader_helper.h"],
    copts = imp_google3_copts(),
    deps = [
        ":editor",
        ":editor_touch",
        ":events",
        ":file_type_loader",
        ":file_type_registry",
        "//core:config",
        "//core/assets",
        "//core/assets/gltf:gltf_behavior_extension",
        "//core/assets/gltf:gltf_interactivity_extension",
        "//core/async:future",
        "//core/common",
        "//core/common:buffer_access",
        "//core/common:invocable",
        "//core/common:registry",
        "//core/editor/widgets:asset_library",
        "//core/lighting:environment_light",
        "//core/lighting:environment_light_factory",
        "//core/lighting:image_based_lighting_asset",
        "//core/lighting:image_based_lighting_asset_iblprefilter_loader",
        "//core/ncsb",
        "//core/resources",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
    ],
    alwayslink = 1,
)

cc_library(
    name = "editor_touch",
    srcs = ["editor_touch.cc"],
    hdrs = ["editor_touch.h"],
    deps = [
        "//core/common:invocable",
        "//core/ncsb",
    ],
    alwayslink = 1,
)

cc_library(
    name = "node_value_command",
    hdrs = [
        "node_value_command.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":command",
        ":editor_touch",
        "//core/common:invocable",
        "//core/ncsb",
        "@com_google_absl//absl/status",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

imp_assets(
    name = "scuba_assets",
    srcs = [
        "//third_party/arcore/googledata/sceneform/assets/gltf_2p0_sample_assets:DamagedHelmet/glTF-Binary/DamagedHelmet.glb",
    ],
    namespace = "imp::editor::scuba_assets",
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

proto_library(
    name = "editor_proto_visitor_test_message_proto",
    srcs = ["editor_proto_visitor_test_message.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
        "//core/proto:test_standard_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

imp_cc_proto_library(
    name = "editor_proto_visitor_test_message_imp_proto",
    proto = ":editor_proto_visitor_test_message_proto",
    deps = [
        "//core/math:math_proto_cc",
        "//core/proto:any_imp_proto",
        "//core/proto:test_standard_cc_proto",
    ],
)

proto_library(
    name = "component_editor_proto",
    srcs = ["component_editor.proto"],
    deps = ["@com_google_protobuf//:any_proto"],
)

imp_cc_proto_library(
    name = "component_editor_cc_proto",
    proto = ":component_editor_proto",
    deps = [
        "//core/proto:any_imp_proto",
    ],
)

cc_library(
    name = "command",
    srcs = [
        "command_manager.cc",
        "function_command.cc",
    ],
    hdrs = [
        "command.h",
        "command_manager.h",
        "function_command.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:invocable",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "editor_plugin",
    hdrs = [
        "editor_plugin.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/editor/layout",
        "//core/view:base_view",
    ],
    alwayslink = 1,
)

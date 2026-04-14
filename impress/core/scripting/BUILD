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
load("//build_tools:imp.bzl", "if_android", "if_desktop", "if_dev_runtime", "if_emscripten", "if_ios", "imp_google3_copts")
load("//core/proto:imp_isf_data.bzl", "imp_isf_data")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "scripting",
    srcs = [
        "scripting_system.cc",
    ],
    hdrs = [
        "scripting_system.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":message_handler",
        ":message_helpers",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:hash",
        "//core/common:invocable",
        "//core/common:jni_helpers",
        "//core/common:rememberer",
        "//core/math",
        "//core/ncsb",
        "//core/proto:any_imp_proto",
        "//core/proto:proto_reader",
        "//core/proto:proto_writer",
        "//core/scripting/proto:bridge_cc_proto",
        "//core/scripting/proto:events_cc_proto",
        "//core/scripting/web:web_view",
        "//core/view:base_view",
        "//core/view/scripting",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ] + if_android([
        "//core/scripting/web:web_view_android",
    ]) + if_ios([
        "//core/scripting/web:web_view_ios",
    ]) + if_desktop([
        "//core/scripting/web:web_view_desktop",
    ]) + if_emscripten([
        "//core/scripting/web:web_view_wasm",
    ]),
    alwayslink = 1,
)

# Provides a helper function to create a ScriptingSystem with the main API.
cc_library(
    name = "basic_api",
    hdrs = [
        "basic_api.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":basic_api_handlers",
        ":scripting",
        "//core:config",
        "//core/view:base_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "scripting_system_web",
    hdrs = [
        "scripting_system_web.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":scripting",
        "//core/common",
        "//core/scripting/web:web_view",
        "//core/view:base_view",
        "//javascript/core:imp_web_js_embed",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "message_handler",
    srcs = ["multi_message_handler.cc"],
    hdrs = [
        "base_message_handler.h",
        "message_handler.h",
        "multi_message_handler.h",
    ],
    deps = [
        ":message_helpers",
        "//core/async:future",
        "//core/proto:any_imp_proto",
        "//core/proto:proto_writer",
        "//core/view/scripting",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "message_helpers",
    hdrs = [
        "message_helpers.h",
    ],
    deps = [
        "//core/proto:any_imp_proto",
        "//core/proto:proto_reader",
        "//core/proto:proto_writer",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "basic_api_handlers",
    srcs = [
        "message_handlers/add_or_update_component_handler.cc",
        "message_handlers/animate_node_handler.cc",
        "message_handlers/create_node_handler.cc",
        "message_handlers/destroy_node_handler.cc",
        "message_handlers/find_node_handler.cc",
        "message_handlers/forward_input_handler.cc",
        "message_handlers/get_animation_names_handler.cc",
        "message_handlers/get_camera_handler.cc",
        "message_handlers/get_children_handler.cc",
        "message_handlers/get_node_active_handler.cc",
        "message_handlers/get_node_name_handler.cc",
        "message_handlers/load_model_handler.cc",
        "message_handlers/play_animation_handler.cc",
        "message_handlers/set_camera_projection_handler.cc",
        "message_handlers/set_node_enabled_handler.cc",
        "message_handlers/set_node_name_handler.cc",
        "message_handlers/set_parent_handler.cc",
        "message_handlers/transform_handler.cc",
    ] + if_dev_runtime([
        "message_handlers/load_file_handler.cc",
    ]) + if_android([
        "message_handlers/load_and_apply_environment_light_handler.cc",
    ]),
    hdrs = [
        "message_handlers/add_or_update_component_handler.h",
        "message_handlers/animate_node_handler.h",
        "message_handlers/create_node_handler.h",
        "message_handlers/destroy_node_handler.h",
        "message_handlers/find_node_handler.h",
        "message_handlers/forward_input_handler.h",
        "message_handlers/get_animation_names_handler.h",
        "message_handlers/get_camera_handler.h",
        "message_handlers/get_children_handler.h",
        "message_handlers/get_node_active_handler.h",
        "message_handlers/get_node_name_handler.h",
        "message_handlers/load_model_handler.h",
        "message_handlers/play_animation_handler.h",
        "message_handlers/set_camera_projection_handler.h",
        "message_handlers/set_node_enabled_handler.h",
        "message_handlers/set_node_name_handler.h",
        "message_handlers/set_parent_handler.h",
        "message_handlers/transform_handler.h",
    ] + if_dev_runtime([
        "message_handlers/load_file_handler.h",
    ]) + if_android([
        "message_handlers/load_and_apply_environment_light_handler.h",
    ]),
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":message_handler",
        ":message_helpers",
        ":scripting",
        "//core:config",
        "//core/assets",
        "//core/async:executor",
        "//core/async:future",
        "//core/async:future_interrupter",
        "//core/common:context",
        "//core/common:enum_flags",
        "//core/common:hash",
        "//core/common:invocable",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/input",
        "//core/input:keyboard_event_cc_proto",
        "//core/input:pointer_event_cc_proto",
        "//core/input:wheel_event_cc_proto",
        "//core/materials",
        "//core/math",
        "//core/model",
        "//core/ncsb",
        "//core/ncsb:node_handle_message_imp_proto",
        "//core/proto:any_imp_proto",
        "//core/proto:proto_writer",
        "//core/scripting/proto:api_cc_proto",
        "//core/scripting/proto:bridge_cc_proto",
        "//core/scripting/proto:events_cc_proto",
        "//core/scripting/web:web_key_codes",
        "//core/scripting/web:web_view",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/scripting",
        "//core/view/utils:asset",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
    ] + if_dev_runtime(
        [
            "//core/editor:events",
            "//core/editor:file_loader_helper",
        ],
    ) + if_android([
        "//core/lighting:environment_light_factory",
        "//core/lighting:image_based_lighting_asset",
        "//core/split_engine:split_engine_serializer",
        "//core/view/platforms/android/wrappers:input_stream",
        "//core/resources:android_url_loader",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "test_lib",
    testonly = True,
    hdrs = [
        "message_handler_test_fixture.h",
    ],
    deps = [
        ":message_handler",
        ":scripting",
        "//core/common:context",
        "//core/ncsb",
        "//testing",
        "//testing:base_generic_test_view",
        "@mediapipe//mediapipe/framework/port:gtest_main",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

imp_assets(
    name = "test_resources",
    srcs = [
        ":load_model_handler_test_scene_isf_data",
        "//third_party/arcore/googledata/sceneform/assets/gltf_2p0_sample_assets:Avocado_glb",
        "//third_party/arcore/googledata/sceneform/assets/gltf_2p0_sample_assets:BoxAnimated_glb",
        "//third_party/arcore/googledata/sceneform/assets/gltf_2p0_sample_assets:Lantern_glb",
        "//third_party/arcore/googledata/sceneform/assets/gltf_2p0_sample_assets:RiggedFigure_glb",
    ],
    namespace = "imp::scripting::test_data",
)

imp_isf_data(
    name = "load_model_handler_test_scene_isf_data",
    srcs = [
        "message_handlers/load_model_handler_test_scene.textproto",
    ],
)

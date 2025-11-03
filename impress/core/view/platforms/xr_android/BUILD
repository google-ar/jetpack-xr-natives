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

load("@bazel_skylib//rules:common_settings.bzl", "string_flag")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "if_dev_runtime", "if_non_android", "imp_copts", "imp_defines", "imp_google3_copts")
load("//build_tools:imp_gltf_materials_package.bzl", "imp_gltf_materials_package")
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

string_flag(
    name = "platform_type",
    build_setting_default = "opengl",
    values = [
        "opengl",
        "vulkan",
    ],
)

config_setting(
    name = "xr_opengl_platform",
    flag_values = {":platform_type": "opengl"},
)

config_setting(
    name = "xr_vulkan_platform",
    flag_values = {":platform_type": "vulkan"},
)

cc_library(
    name = "openxr_includes",
    hdrs = [
        "openxr_includes.h",
    ],
    defines = imp_defines(),
    deps = [
        "//core:config",
        "@third_party//OpenXR_KHR:openxr_headers_androidxr",
        "@third_party//filament/filament/backend:platform",
    ] + select({
        ":xr_opengl_platform": [
            "@third_party//filament/filament/backend:backend_hdrs",
            "@third_party//filament/filament/backend:backend_opengl_private_hdrs",
            "@third_party//gl",
        ],
        ":xr_vulkan_platform": [
            "@third_party//filament/filament/backend:backend_vk_hdrs",
            "@third_party//filament/libs:bluevk_hdrs",
        ],
    }) + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "xr",
    srcs = [
        "xr_session_host.cc",
        "xr_swap_chain.cc",
    ] + select({
        ":xr_opengl_platform": [
            "xr_opengl_platform.cc",
            "xr_opengl_swap_chain_image_handler.cc",
        ],
        ":xr_vulkan_platform": [
            "xr_vulkan_platform.cc",
            "xr_vulkan_swap_chain_image_handler.cc",
        ],
    }),
    hdrs = [
        "xr_session_host.h",
        "xr_swap_chain.h",
    ] + select({
        ":xr_opengl_platform": [
            "xr_opengl_platform.h",
            "xr_opengl_swap_chain_image_handler.h",
        ],
        ":xr_vulkan_platform": [
            "xr_vulkan_platform.h",
            "xr_vulkan_swap_chain_image_handler.h",
        ],
    }),
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":generic_stereo_materials",
        ":openxr_includes",
        ":xr_color_space_helper",
        ":xr_events_proto_cc",
        ":xr_helpers",
        "//core:config",
        "//core/actions:action_config",
        "//core/actions:controller_events",
        "//core/actions:input_action_event",
        "//core/async:executor",
        "//core/common:enum_flags",
        "//core/common:invocable",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/common:trace",
        "//core/math",
        "//core/monitor",
        "//core/monitor:default_monitor_summary",
        "//core/monitor:monitor_summary",
        "//core/monitor:profiling_clock",
        "//core/monitor:simple_histogram",
        "//core/ncsb",
        "//core/render:content_security_level",
        "//core/view:base_view",
        "//core/view:view_host",
        "//core/view/framework",
        "//core/window:filament_host",
        "//core/xr:openxr_events",
        "//core/xr:openxr_loader_lib",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/deps:clock",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:platform",
        "@third_party//filament/libs:utils_hdrs",
    ] + select({
        ":xr_opengl_platform": [
            "@third_party//filament/filament/backend:backend_opengl_private_hdrs",
            "@third_party//gl",
        ],
        ":xr_vulkan_platform": [
            "@third_party//filament/filament/backend:backend_vk_hdrs",
            "@third_party//filament/libs:bluevk_hdrs",
        ],
    }) + if_non_android(["@bazel_tools//tools/jdk:jni"]) + if_dev_runtime([":xr_editor_plugin"]),
    alwayslink = 1,
)

cc_library(
    name = "xr_color_space_helper",
    srcs = [
        "xr_color_space_helper.cc",
    ],
    hdrs = [
        "xr_color_space_helper.h",
    ],
    defines = imp_defines(),
    deps = [
        "//core/assets/material:material_load_options_proto_cc",
        "//core/common:rememberer",
        "//core/math",
        "//core/ncsb",
        "//core/render:display_color_space",
        "//core/view:base_view",
        "//core/view:view_events",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "xr_helpers",
    srcs = [
        "xr_helpers.cc",
    ],
    hdrs = [
        "xr_helpers.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":openxr_includes",
        "//core/math",
        "//core/monitor",
        "//core/monitor:monitor_summary",
        "//core/view:base_view",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "xr_jni",
    srcs = [
        "xr_jni.cc",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":openxr_includes",
        ":xr",
        ":xr_action_controller",
        ":xr_helpers",
        "//core/common:jni_helpers",
        "//core/monitor",
        "//core/render:content_security_level",
        "//core/view/framework",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//gl",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

proto_library(
    name = "xr_events_proto",
    srcs = ["xr_events.proto"],
    deps = [
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "xr_events_proto_cc",
    proto = ":xr_events_proto",
)

java_lite_proto_library(
    name = "xr_events_java_proto_lite",
    deps = [":xr_events_proto"],
)

cc_library(
    name = "xr_action_controller",
    srcs = ["xr_action_controller.cc"],
    hdrs = [
        "xr_action_controller.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":openxr_includes",
        ":xr",
        ":xr_action_config",
        ":xr_action_params_imp_cc_proto",
        ":xr_helpers",
        "//core/actions:action_config",
        "//core/actions:controller_input_handler",
        "//core/actions:input_action_event",
        "//core/common:robin_map",
        "//core/common:trace",
        "//core/math",
        "//core/view:base_view",
        "//core/view/utils:string_hasher",
        "//core/view/utils:string_map",
        "//core/xr:openxr_events",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "xr_action_config",
    srcs = [
        "xr_action_config.cc",
    ],
    hdrs = [
        "xr_action_config.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":xr_action_params_imp_cc_proto",
        "//core/actions:action_config",
    ],
    alwayslink = 1,
)

cc_library(
    name = "xr_editor_plugin",
    srcs = [
        "xr_editor_plugin.cc",
    ],
    hdrs = ["xr_editor_plugin.h"],
    deps = [
        "//core/editor:editor_constants",
        "//core/editor:editor_plugin",
        "//core/editor:events",
        "//core/editor/layout",
        "//core/editor/layout:layout_config_imp_cc_proto",
        "//core/editor/xr:xr_editor_ui",
        "//core/ncsb",
        "//core/view:base_view",
    ],
    alwayslink = 1,
)

proto_library(
    name = "xr_action_params_proto",
    srcs = ["xr_action_params.proto"],
)

imp_cc_proto_library(
    name = "xr_action_params_imp_cc_proto",
    proto = ":xr_action_params_proto",
)

imp_gltf_materials_package(
    name = "imp_default_stereo_gltf_materials",
    blend_modes = [
        "masked",
        "opaque",
        "transparent",
        "refractive",
    ],
    features = [
        "clearcoat",
        "sheen",
        "transmission",
        "stereo",
    ],
    optimization = "performance",
    shading_models = [
        "lit",
        "unlit",
    ],
    sided = [
        "single_sided",
        "double_sided",
    ],
)

imp_assets(
    name = "generic_stereo_materials",
    srcs = ["//core/view/platforms/xr_android:compiled_imp_default_stereo_gltf_materials.zip"],
    namespace = "imp::materials",
)

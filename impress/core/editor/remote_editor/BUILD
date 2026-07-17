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
    "//build_tools:imp.bzl",
    "if_android",
    "if_non_android",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "remote_editor_jni",
    srcs = [
        "remote_editor_script_api_bridge_jni.cc",
        "remote_editor_video_streamer_jni.cc",
    ],
    linkstatic = 1,
    deps = [
        ":remote_editor",
        "//core:config",
        "//core/async:executor",
        "//core/common:buffer_access",
        "//core/common:jni_helpers",
        "//core/common:registry",
        "//core/proto:proto_reader",
        "//core/scripting",
        "//core/scripting:message_helpers",
        "//core/scripting/proto:bridge_cc_proto",
        "//core/view:base_view",
        "//core/view/scripting",
        "@com_google_absl//absl/base",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "remote_editor_renderer",
    srcs = [
        "remote_editor_renderer.cc",
    ],
    hdrs = [
        "remote_editor_renderer.h",
    ],
    linkstatic = 1,
    deps = [
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:future",
        "//core/common:rememberer",
        "//core/editor/components:world_space_editor_ui_assets",
        "//core/materials",
        "//core/math",
        "//core/ncsb",
        "//core/render",
        "//core/render:primitive_shape_renderer",
        "//core/render:primitive_shape_renderer_state_cc_proto",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "//core/window:filament_host",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "remote_editor",
    srcs = [
        "noop_remote_editor.h",
        "remote_editor.cc",
        "web_ui_remote_editor.cc",
    ] + if_android([
        "android_web_ui_remote_editor.cc",
        "android_remote_editor_script_api_bridge_wrapper.cc",
        "android_remote_editor_wrapper.cc",
    ]),
    hdrs = [
        "remote_editor.h",
        "web_ui_remote_editor.h",
    ] + if_android([
        "android_web_ui_remote_editor.h",
        "android_remote_editor_wrapper.h",
        "android_remote_editor_script_api_bridge_wrapper.h",
    ]),
    linkstatic = 1,
    deps = [
        ":remote_editor_info",
        ":remote_editor_renderer",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:jni_helpers",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/editor",
        "//core/editor:editor_constants",
        "//core/editor:editor_info",
        "//core/editor:widget_ui_system",
        "//core/editor/components:world_space_editor_ui_assets",
        "//core/editor/layout",
        "//core/scripting",
        "//core/scripting:basic_api",
        "//core/scripting:basic_api_handlers",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "remote_editor_info",
    hdrs = ["remote_editor_info.h"],
    alwayslink = 1,
)

# The remote editor has an Android-specific implementation that uses JNI to
# communicate with Android system APIs. remote_editor_test.cc tests this
# implementation when built for Android, so it needs to be run in a
# simulated Android environment. cc_test_on_robolectric runs the test using
# Robolectric, which provides this simulated environment.

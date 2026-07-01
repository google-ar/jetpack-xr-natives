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

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "remote_editor_jni",
    srcs = if_android([
        "remote_editor_script_api_bridge_jni.cc",
        "remote_editor_video_streamer_jni.cc",
    ]),
    linkstatic = 1,
    deps = [
        ":android_remote_editor",
        "//core:config",
        "//core/async:executor",
        "//core/common:buffer_access",
        "//core/common:jni_helpers",
        "//core/common:registry",
        "//core/editor/remote_editor",
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
    name = "android_remote_editor",
    srcs = if_android([
        "android_remote_editor_script_api_bridge_wrapper.cc",
        "android_remote_editor_wrapper.cc",
        "android_web_ui_remote_editor.cc",
    ]),
    hdrs = if_android([
        "android_remote_editor_script_api_bridge_wrapper.h",
        "android_remote_editor_wrapper.h",
        "android_web_ui_remote_editor.h",
    ]),
    linkstatic = 1,
    deps = [
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:jni_helpers",
        "//core/editor",
        "//core/editor:editor_info",
        "//core/editor/remote_editor:remote_editor_base",
        "//core/editor/remote_editor:remote_editor_info",
        "//core/editor/remote_editor:remote_editor_renderer",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view:view_events",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

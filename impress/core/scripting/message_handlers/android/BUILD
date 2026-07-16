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
load("//build_tools:imp.bzl", "if_non_android", "imp_copts")
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
    name = "android_view_renderer",
    srcs = [
        "android_view_renderer.cc",
    ],
    hdrs = [
        "android_view_renderer.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":android_view_renderer_proto_cc",
        "//core:config",
        "//core/actions:action_config",
        "//core/actions:controller_events",
        "//core/assets",
        "//core/async:executor",
        "//core/async:future",
        "//core/collision:ray",
        "//core/common",
        "//core/common:context",
        "//core/common:filament_helpers",
        "//core/common:jni_helpers",
        "//core/geometry/shapes",
        "//core/input",
        "//core/materials",
        "//core/math",
        "//core/ncsb",
        "//core/render",
        "//core/render:primitive_shape_renderer",
        "//core/render:primitive_shape_renderer_state_cc_proto",
        "//core/render/android:android_external_texture_surface",
        "//core/split_engine/materials/builtin:builtin_texture_external_material",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/platforms/android/wrappers:graphics",
        "//core/view/platforms/android/wrappers:motion_event",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@third_party//split_engine/input:split_engine_input_event",
        "@third_party//split_engine/materials:texture_external_material",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "android_view_request_handler",
    srcs = [
        "android_view_request_handler.cc",
    ],
    hdrs = [
        "android_view_request_handler.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":android_view_renderer",
        ":android_view_renderer_proto_cc",
        "//core:config",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/math",
        "//core/ncsb",
        "//core/scripting:message_handler",
        "//core/view:base_view",
        "//core/view/scripting",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

proto_library(
    name = "android_view_renderer_proto",
    srcs = ["android_view_renderer.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/ncsb:node_handle_message_proto",
        "//core/proto:imp_proto",
        "//core/view/framework:framework_proto",
    ],
)

imp_cc_proto_library(
    name = "android_view_renderer_proto_cc",
    proto = ":android_view_renderer_proto",
    deps = [
        "//core/ncsb:node_handle_message_imp_proto",
        "//core/proto:any_imp_proto",
        "//core/view/framework:framework_proto_cc",
    ],
)

java_lite_proto_library(
    name = "android_view_renderer_java_proto_lite",
    deps = [":android_view_renderer_proto"],
)

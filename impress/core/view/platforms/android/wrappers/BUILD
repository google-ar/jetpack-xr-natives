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
load("//build_tools:imp.bzl", "if_non_android", "imp_copts", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "imp_lifecycle_callback",
    srcs = ["imp_lifecycle_callback.cc"],
    hdrs = ["imp_lifecycle_callback.h"],
    copts = imp_copts(),
    deps = [
        "//core/common:jni_helpers",
        "//core/view:base_view",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "surface_texture",
    srcs = [
        "surface_texture.cc",
    ],
    hdrs = [
        "surface_texture.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/math",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "graphics",
    srcs = [
        "bitmap.cc",
        "canvas.cc",
        "font.cc",
        "font_style.cc",
        "graphics_helpers.cc",
        "paint.cc",
        "picture.cc",
        "rect.cc",
        "surface.cc",
        "typeface.cc",
    ],
    hdrs = [
        "bitmap.h",
        "canvas.h",
        "font.h",
        "font_style.h",
        "graphics_helpers.h",
        "paint.h",
        "picture.h",
        "rect.h",
        "surface.h",
        "typeface.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":file",
        ":surface_texture",
        "//core:config",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/geometry/shapes",
        "//core/math",
        "//core/render:content_security_level",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "motion_event",
    srcs = ["motion_event.cc"],
    hdrs = ["motion_event.h"],
    deps = [
        "//core/common:jni_helpers",
        "//core/common:platform_helpers",
        "//core/input",
        "//core/math",
        "@com_google_absl//absl/status:statusor",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "activity_context",
    srcs = ["activity_context.cc"],
    hdrs = ["activity_context.h"],
    deps = [
        ":file",
        "//core:config",
        "//core/common:jni_helpers",
        "@com_google_absl//absl/strings",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "file",
    srcs = ["file.cc"],
    hdrs = ["file.h"],
    deps = [
        "//core/common:jni_helpers",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "input_stream",
    srcs = [
        "input_stream.cc",
    ],
    hdrs = ["input_stream.h"],
    deps = [
        ":activity_context",
        ":file",
        "//core:config",
        "//core/async:future_interrupter",
        "//core/common:jni_helpers",
        "//core/monitor:profiling_clock",
        "//core/resources:download_progress_info",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

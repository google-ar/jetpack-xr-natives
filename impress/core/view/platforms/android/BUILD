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

load("@bazel_skylib//:bzl_library.bzl", "bzl_library")
load("//build_tools:imp.bzl", "if_android", "if_non_android", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

# Library for linking in Impress jni and Filament jni + implementation on android.
#
# An incomplete library which is combined via .bzl with a provider of the client API.
cc_library(
    name = "android",
    copts = imp_google3_copts(),
    deps = [
        ":android_no_filament",
        "@third_party//filament",
    ] + if_android(
        [
            "@third_party//gl",
        ],
    ) + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

# Library for linking in Impress Jni on android without bringing in filament jni or implementation.
# Use this library if filament is being linked in independently to avoid duplicate symbols.
#
# An incomplete library which is combined via .bzl with a provider of the client API.
cc_library(
    name = "android_no_filament",
    srcs = [
        "input_jni.cc",
        "keyboard_view_jni.cc",
        "view_jni.cc",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/async:executor",
        "//core/common",
        "//core/common:context",
        "//core/common:filament_engine_helpers",
        "//core/common:jni_helpers",
        "//core/common:log",
        "//core/common:platform_helpers",
        "//core/common:trace",
        "//core/input",
        "//core/math",
        "//core/render_passes:surface_renderer",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view:view_host",
        "//core/view/framework",
        "//core/view/platforms/android/wrappers:imp_lifecycle_callback",
        "//core/window:filament_host",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:platform",
        "@third_party//filament/filament/backend:virtual_machine_env",
    ],
    alwayslink = 1,
)

bzl_library(
    name = "generate_activity_bzl",
    srcs = ["generate_activity.bzl"],
    # TODO: Restore visibility restriction.
    #visibility = ["//visibility:private"],
)

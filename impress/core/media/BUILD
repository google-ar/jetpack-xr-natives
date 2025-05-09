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

load("//build_tools:imp.bzl", "if_non_android", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "media",
    srcs = ["media_asset.cc"],
    hdrs = [
        "media_asset.h",
        "media_source.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/async:future",
        "//core/common:trace",
        "//core/resources",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "media_type",
    srcs = ["media_type.cc"],
    hdrs = [
        "media_type.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common",
        "//core/common:robin_map",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "android_media",
    srcs = [
        "android/android_media_data_source.h",
        "android/android_media_player.h",
        "android/android_media_source.cc",
        "android/android_playback_params.h",
    ],
    hdrs = [
        "android/android_media_source.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":android_media_jni",
        ":media",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/view:base_view",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "android_media_jni",
    srcs = ["android/android_media_listener.cc"],
    hdrs = ["android/android_media_listener.h"],
    deps = [
        "//core/common:jni_helpers",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "android_exoplayer_media",
    srcs = [],
    hdrs = [
        "android/android_exoplayer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":android_exoplayer_media_jni",
        ":media",
        ":media_type",
        "//core/async:future",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/video:video_color_space",
        "//core/view:base_view",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "android_exoplayer_media_jni",
    srcs = ["android/android_exoplayer_listener.cc"],
    hdrs = ["android/android_exoplayer_listener.h"],
    deps = [
        "//core/common:context",
        "//core/common:jni_helpers",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

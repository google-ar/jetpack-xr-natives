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

load("@bazel_skylib//lib:selects.bzl", "selects")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "if_android_min_sdk_at_least_26", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

selects.config_setting_group(
    name = "mvhevc_on_android",
    match_all = [
        "//core:android",
        "//core:imp_android_external_texture_surface_uses_image_reader",
    ],
)

# Wrapper library for Android NDK AImage.
cc_library(
    name = "image",
    srcs = select({
        "//core:android": ["image.cc"],
        "//conditions:default": [],
    }),
    hdrs = select({
        "//core:android": ["image.h"],
        "//conditions:default": [],
    }),
    copts = imp_google3_copts(),
    linkopts = if_android_min_sdk_at_least_26([
        "-lmediandk",
        "-lnativewindow",
    ]),
    deps = [
        "@mediapipe//mediapipe/framework/port:status",
    ] + select({
        "//core:android": [
            "//core:config",
            "//core/common:robin_map",
            "//core/math",
            "//core/render/android:definitions",
            "@com_google_absl//absl/cleanup",
            "@com_google_absl//absl/status",
            "@com_google_absl//absl/status:statusor",
            "@com_google_absl//absl/strings",
            "@com_google_impress//core/common:log",
        ],
        "//conditions:default": [],
    }),
    alwayslink = 1,
)

# Wrapper library for Android NDK AImageReader.
cc_library(
    name = "image_reader",
    srcs = select({
        "//core:android": ["image_reader.cc"],
        "//conditions:default": [],
    }),
    hdrs = select({
        "//core:android": ["image_reader.h"],
        "//conditions:default": [],
    }),
    copts = imp_google3_copts(),
    linkopts = if_android_min_sdk_at_least_26([
        "-lmediandk",
        "-lnativewindow",
    ]),
    deps = select({
        "//core:android": [
            ":image",
            "//core:config",
            "//core/common:context",
            "//core/view:base_view",
            "@com_google_absl//absl/log:check",
            "@com_google_absl//absl/status",
            "@com_google_absl//absl/status:statusor",
            "@com_google_absl//absl/strings",
            "@mediapipe//mediapipe/framework/port:status",
        ],
        "//conditions:default": [],
    }) + [
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

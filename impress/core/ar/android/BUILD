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
load("//build_tools:imp.bzl", "imp_google3_copts")
load(
    "//third_party/arcore/build_defs:flavored_rules.bzl",
    "add_flavor_name",
)

# Library for providing ARCore tracking.

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

# Creates a custom build flag
# Enables the following:
# "blaze --blazerc=third_party/arcore/blazerc build --//core/ar/android:impress_flavor="experimental" //core/ar/android:ar_core_ptrs"
string_flag(
    name = "impress_flavor",
    build_setting_default = "release",
    values = [
        "release",
        "experimental",
    ],
)

config_setting(
    name = "impress_experimental",
    constraint_values = [
        "@platforms//os:android",
    ],
    flag_values = {
        "//core/ar/android:impress_flavor": "experimental",
    },
)

cc_library(
    name = "ar_core_ptrs",
    hdrs = ["ar_core_ptrs.h"],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = select({
        ":impress_experimental": [
            add_flavor_name(
                "//third_party/arcore/ar/core/c_api",
                flavor = "experimental",
            ),
        ],
        "//conditions:default": [
            add_flavor_name(
                "//third_party/arcore/ar/core/c_api",
                flavor = "release",
            ),
        ],
    }),
    alwayslink = 1,
)

cc_library(
    name = "android",
    srcs = [
        "ar_session_native_arcore.cc",
    ],
    hdrs = [
        "ar_session_native_arcore.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":android_camera_texture",
        ":ar_core_ptrs",
        ":deeplight_controller",
        ":depth_texture_controller",
        "//core/ar:ar_hdr_lighting",
        "//core/ar:ar_session_config",
        "//core/ar:shared",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:filament_helpers",
        "//core/common:trace",
        "//core/lighting:environment_light_factory",
        "//core/math",
        "//core/render",
        "//core/view:base_view",
        "//third_party/arcore/ar/core/c_api:pose",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//gl",
    ] + select({
        ":impress_experimental": [
            add_flavor_name(
                "//third_party/arcore/ar/core/c_api",
                flavor = "experimental",
            ),
        ],
        "//conditions:default": [
            add_flavor_name(
                "//third_party/arcore/ar/core/c_api",
                flavor = "release",
            ),
        ],
    }),
    alwayslink = 1,
)

cc_library(
    name = "deeplight_controller",
    srcs = ["deeplight_controller.cc"],
    hdrs = ["deeplight_controller.h"],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":android_camera_texture",
        ":ar_core_ptrs",
        "//core/ar:ar_hdr_lighting",
        "//core/common",
        "//core/common:trace",
        "//core/lighting:environment_light_factory",
        "//core/math",
        "//core/render",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:generatePrefilterMipmap",
        "@third_party//filament/libs:math_hdrs",
        add_flavor_name(
            "//third_party/arcore/ar/core/c_api",
            flavor = "release",
        ),
    ],
    alwayslink = 1,
)

cc_library(
    name = "depth_texture_controller",
    srcs = ["depth_texture_controller.cc"],
    hdrs = ["depth_texture_controller.h"],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":ar_core_ptrs",
        "//core/common:trace",
        "//core/render",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        add_flavor_name(
            "//third_party/arcore/ar/core/c_api",
            flavor = "release",
        ),
    ],
    alwayslink = 1,
)

cc_library(
    name = "android_camera_texture",
    srcs = ["android_camera_texture.cc"],
    hdrs = ["android_camera_texture.h"],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        "//core:config",
        "//core/ar:shared",
        "//core/common:platform_helpers",
        "//core/common:robin_map",
        "//core/math",
        "//core/render",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//gl",
    ],
    alwayslink = 1,
)

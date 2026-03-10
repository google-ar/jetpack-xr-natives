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
load("@rules_cc//cc:objc_library.bzl", "objc_library")
load("//build_tools:imp.bzl", "if_linux", "if_macos", "imp_copts")
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
    name = "font_holder",
    hdrs = [
        "font_holder.h",
    ],
    copts = imp_copts(),
    deps = [
        ":font_params_proto_cc",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "android_font_holder",
    srcs = [
        "android_font_font_holder.cc",
        "android_typeface_font_holder.cc",
    ],
    hdrs = [
        "android_font_font_holder.h",
        "android_typeface_font_holder.h",
    ],
    copts = imp_copts(),
    deps = [
        ":font_holder",
        ":font_params_proto_cc",
        "//core/common",
        "//core/view/platforms/android/wrappers:file",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

objc_library(
    name = "ios_font_holder",
    srcs = [
        "ios_font_holder.mm",
    ],
    hdrs = [
        "ios_font_holder.h",
    ],
    deps = [
        ":font_holder",
        ":font_params_proto_cc",
        "//third_party/apple_frameworks:UIKit",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "wasm_font_holder",
    srcs = ["wasm_font_holder.cc"],
    hdrs = ["wasm_font_holder.h"],
    copts = imp_copts(),
    deps = [
        ":font_holder",
        ":font_params_proto_cc",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "desktop_font_holder",
    srcs = ["desktop_font_holder.cc"],
    hdrs = ["desktop_font_holder.h"],
    copts = imp_copts(),
    data = if_linux([
        "//core/fonts:googlesans_arabic_medium.ttf",
        "//core/fonts:googlesans_japanese_medium.ttf",
        "//core/fonts:googlesans_medium.ttf",
    ]),
    deps = [
        ":font_holder",
        ":font_params_proto_cc",
        "//core:config",
        "//core/common:platform_helpers",
        "//third_party/skia/HEAD:core",
        "//third_party/skia/HEAD:skparagraph_harfbuzz_skunicode",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
    ] + if_linux([
        "@com_google_absl//absl/debugging:leak_check",
        "//third_party/fontconfig",
        "//third_party/skia/HEAD:fontmgr_fontconfig",
        "//third_party/skia/HEAD:freetype_support",
    ]) + if_macos([
        "//third_party/skia/HEAD:fontmgr_coretext",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "system_font_provider",
    srcs = [
        "system_font_provider.cc",
    ],
    hdrs = [
        "system_font_provider.h",
    ],
    copts = imp_copts(),
    deps = [
        ":font_holder",
        ":font_params_proto_cc",
        "//core:config",
        "//core/common:context",
        "//core/common:platform_helpers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings",
    ] + select({
        "//core:android": [
            ":android_system_font_provider",
        ],
        "//core:ios": [
            ":ios_system_font_provider",
        ],
        "//core:emscripten": [
            ":wasm_system_font_provider",
        ],
        "//conditions:default": [
            ":desktop_system_font_provider",
        ],
    }),
    alwayslink = 1,
)

cc_library(
    name = "android_system_font_provider",
    srcs = [
        "android_system_font_provider.cc",
    ],
    hdrs = [
        "android_system_font_provider.h",
    ],
    copts = imp_copts(),
    deps = [
        ":android_font_holder",
        ":font_holder",
        ":font_params_proto_cc",
        "//core/common:context",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "desktop_system_font_provider",
    srcs = ["desktop_system_font_provider.cc"],
    hdrs = ["desktop_system_font_provider.h"],
    deps = [
        ":desktop_font_holder",
        ":font_holder",
        ":font_params_proto_cc",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

objc_library(
    name = "ios_system_font_provider",
    srcs = [
        "ios_system_font_provider.mm",
    ],
    hdrs = [
        "ios_system_font_provider.h",
    ],
    deps = [
        ":font_holder",
        ":font_params_proto_cc",
        ":ios_font_holder",
        "//core/common:context",
        "//third_party/apple_frameworks:UIKit",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "wasm_system_font_provider",
    srcs = ["wasm_system_font_provider.cc"],
    hdrs = ["wasm_system_font_provider.h"],
    copts = imp_copts(),
    deps = [
        ":font_holder",
        ":font_params_proto_cc",
        ":wasm_font_holder",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "font_params_helper",
    hdrs = ["font_params_helper.h"],
    deps = [
        ":font_params_proto_cc",
        "@com_google_absl//absl/hash",
    ],
    alwayslink = 1,
)

proto_library(
    name = "font_params_proto",
    srcs = ["font_params.proto"],
)

imp_cc_proto_library(
    name = "font_params_proto_cc",
    proto = ":font_params_proto",
)

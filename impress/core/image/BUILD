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
load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "if_android", "if_non_android", "imp_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "image",
    srcs = [
        "image_contents.cc",
    ],
    hdrs = [
        "image_contents.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        "//core/common",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "wasm_texture_contents",
    hdrs = ["wasm_texture_contents.h"],
    copts = imp_copts(),
    linkstatic = True,
    alwayslink = 1,
)

cc_library(
    name = "wrapped_image_contents",
    srcs = [
        "wrapped_image_contents.cc",
    ],
    hdrs = [
        "wrapped_image_contents.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image",
        "@com_google_absl//absl/types:span",
        "@third_party//filament/filament/backend:backend_utils",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "inline_image_contents",
    srcs = [
        "inline_image_contents.cc",
    ],
    hdrs = [
        "inline_image_contents.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image",
        "//core/common:log",
        "@com_google_absl//absl/log:check",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "android_bitmap_image_contents",
    srcs = if_android([
        "android_bitmap_image_contents.cc",
    ]),
    hdrs = [
        "android_bitmap_image_contents.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image",
        "//core:config",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/common:log",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/status:statusor",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "bitmap_factory_decode_image",
    srcs = if_android([
        "bitmap_factory_decode_image.cc",
    ]),
    hdrs = [
        "bitmap_factory_decode_image.h",
    ],
    copts = imp_copts(),
    linkopts = [
        "-ljnigraphics",
    ],
    linkstatic = True,
    deps = [
        ":android_bitmap_image_contents",
        ":image",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/resources",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "ndk_decode_image",
    srcs = if_android([
        "ndk_decode_image.cc",
    ]),
    hdrs = [
        "ndk_decode_image.h",
    ],
    copts = imp_copts(),
    linkopts = [
        "-ljnigraphics",
    ],
    linkstatic = True,
    deps = [
        ":image",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/resources",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

# Note that the imp_app rule adds the `-s ALLOW_MEMORY_GROWTH=1` linkopt flag. If you are depending
# on this library without the imp_app rule, ensure that you have this linkopt flag.
cc_library(
    name = "wasm_decode_image",
    srcs = ["wasm_decode_image.cc"],
    hdrs = ["wasm_decode_image.h"],
    copts = imp_copts(),
    linkopts = [
        "--post-js $(location //javascript/core/wasm/decode_image:decode_image.js.lds)",
    ],
    tags = ["nobuilder"],
    deps = [
        ":image",
        ":wasm_texture_contents",
        "//base:atomic_sequence_num",
        "//core/async:future",
        "//core/common:log",
        "//core/resources",
        "//javascript/core/wasm/decode_image:decode_image.js.lds",
        "//third_party/emscripten:GL-webgl2-full_es3",
        "//third_party/emscripten:embind",
        "@com_google_absl//absl/base:no_destructor",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "stb_decode_image",
    srcs = [
        "stb_decode_image.cc",
    ],
    hdrs = [
        "stb_decode_image.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image",
        "//core/common",
        "//core/resources",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@stblib//:stb_image",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "webp_decode_image",
    hdrs = [
        "webp_decode_image.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image",
        "//core/resources",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "webp_decode_image_noop",
    srcs = [
        "webp_decode_image_noop.cc",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image",
        ":webp_decode_image",
        "//core/resources",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "wimage_decode_image",
    srcs = [
        "wimage_decode_image.cc",
    ],
    hdrs = [
        "wimage_decode_image.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image",
        "//core/common",
        "//core/resources",
        "//image/wimage",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

# Decodes a glTF2-compatible Ktx2 into its compressed texture format.
cc_library(
    name = "ktx2_decode_image",
    srcs = ["ktx2_decode_image.cc"],
    hdrs = ["ktx2_decode_image.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image",
        ":ktx2_file",
        "//core/common",
        "//core/loader:loader_options",
        "@basis_universal//:basisu_headers_lib",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/types:variant",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament/backend:driver_enums",
        "@zstdlib",
    ] +
    # TODO: These libraries are incompatible. Make them work together.
    selects.with_or({
        (
            "//core:ios",
            "//core:android",
            "//core:emscripten",
        ): ["@basis_universal//:basisu_transcoder_etc2_astc"],
        "//conditions:default": ["@basis_universal//:basis_universal"],
    }),
    alwayslink = 1,
)

# C++ library for reading .ktx2 files.
cc_library(
    name = "ktx2_file",
    srcs = ["ktx2_file.cc"],
    hdrs = ["ktx2_file.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "decode_image",
    srcs = [
        "decode_image.cc",
    ],
    hdrs = [
        "decode_image.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image",
        "//core:config",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:log",
        "//core/resources",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ] + select({
        "//core:android": [
            ":bitmap_factory_decode_image",
            ":ndk_decode_image",
        ],
        "//core:ios": [
            ":stb_decode_image",
            ":webp_decode_image",
        ],
        "//core:emscripten": [
            ":wasm_decode_image",
        ],
        "//conditions:default": [":stb_decode_image"],
    }),
    alwayslink = 1,
)

imp_assets(
    name = "lena_test_resource",
    srcs = ["test_data/lena.png"],
    namespace = "imp::test_data",
)

# --android_ndk_min_sdk_version=23

# --config=ios_x86_64

# --android_ndk_min_sdk_version=23

# --config=ios_x86_64

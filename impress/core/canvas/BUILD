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

load("//build_tools:imp.bzl", "imp_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "constants",
    hdrs = ["constants.h"],
    copts = imp_copts(),
    alwayslink = 1,
)

cc_library(
    name = "async_canvas_source",
    hdrs = ["async_canvas_source.h"],
    copts = imp_copts(),
    deps = [
        ":canvas_interfaces",
        "//core/async:future",
        "//core/canvas/fonts:font_holder",
        "//core/common:small_source_location",
        "//core/math",
        "//core/text:text_helpers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "async_canvas_source_factory",
    srcs = ["async_canvas_source_factory.cc"],
    hdrs = ["async_canvas_source_factory.h"],
    copts = imp_copts(),
    deps = [
        ":async_canvas_source",
        ":canvas_interfaces",
        ":canvas_source",
        "//core:config",
        "//core/async:future",
        "//core/canvas/fonts:font_holder",
        "//core/math",
        "//core/view:base_view",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
    ] + select({
        "//core:emscripten": [
            ":wasm_async_canvas_source",
        ],
        "//conditions:default": [
            ":async_canvas_source_wrapper",
        ],
    }),
    alwayslink = 1,
)

cc_library(
    name = "async_canvas_source_wrapper",
    srcs = ["async_canvas_source_wrapper.cc"],
    hdrs = ["async_canvas_source_wrapper.h"],
    copts = imp_copts(),
    deps = [
        ":async_canvas_source",
        ":async_scoped_canvas_wrapper",
        ":canvas_interfaces",
        ":canvas_source",
        "//core:config",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/math",
        "//core/text:text_helpers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:variant",
    ],
    alwayslink = 1,
)

cc_library(
    name = "canvas_source",
    srcs = [
        "canvas_source.cc",
    ] + select({
        "//core:android": [
            "android_glyph_source.cc",
            "android_glyph_source.h",
            "android_platform_canvas_source.cc",
            "android_platform_canvas_source.h",
        ],
        "//core:ios": [
        ],
        "//core:emscripten": [
            "wasm_platform_canvas_source.cc",
            "wasm_platform_canvas_source.h",
        ],
        "//conditions:default": [
            "desktop_platform_canvas_source.cc",
            "desktop_platform_canvas_source.h",
        ],
    }),
    hdrs = [
        "canvas_source.h",
    ],
    copts = imp_copts(),
    tags = [
        "ignore_for_dep=third_party/skia/HEAD/src/ports/SkFontMgr_custom.h",
    ],
    deps = [
        ":canvas_interfaces",
        ":constants",
        "//:api",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/canvas/fonts:font_holder",
        "//core/canvas/fonts:font_params_proto_cc",
        "//core/common:buffer_access",
        "//core/common:context",
        "//core/common:invocable",
        "//core/common:log",
        "//core/common:platform_helpers",
        "//core/common:resource_helpers",
        "//core/common:small_source_location",
        "//core/geometry/shapes",
        "//core/math",
        "//core/render",
        "//core/text:text_helpers",
        "//core/view:base_view",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:optional",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ] + select({
        "//core:android": [
            "//core/canvas/fonts:android_font_holder",
            "//core/common:jni_helpers",
            "//core/view/platforms/android/wrappers:graphics",
            "//core/view/platforms/android/wrappers:surface_texture",
        ],
        "//core:ios": [
            ":ios_platform_canvas_source",
        ],
        "//core:emscripten": [
            ":async_canvas_source",
            ":wasm_async_canvas_source",
            ":wasm_platform_canvas_source",
            "//base:atomic_sequence_num",
        ],
        "//conditions:default": [
            "//core/canvas/fonts:desktop_font_holder",
            "//third_party/skia/HEAD:core",
            "//third_party/skia/HEAD:skparagraph_harfbuzz_skunicode",
            "//third_party/skia/HEAD:skunicode_icu",
            "//util/utf8/public:unicodetext",
            "@com_google_absl//absl/debugging:leak_check",
        ],
    }),
    alwayslink = 1,
)

cc_library(
    name = "canvas_interfaces",
    hdrs = [
        "async_scoped_canvas.h",
        "platform_canvas_source.h",
        "scoped_canvas.h",
    ],
    copts = imp_copts(),
    deps = [
        ":constants",
        "//core/async:future",
        "//core/canvas/fonts:font_holder",
        "//core/common:invocable",
        "//core/common:small_source_location",
        "//core/geometry/shapes",
        "//core/math",
        "//core/render",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "async_scoped_canvas_wrapper",
    srcs = ["async_scoped_canvas_wrapper.cc"],
    hdrs = ["async_scoped_canvas_wrapper.h"],
    copts = imp_copts(),
    deps = [
        ":canvas_interfaces",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/geometry/shapes",
        "//core/math",
        "//core/render",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "wasm_platform_canvas_source",
    hdrs = ["wasm_platform_canvas_source.h"],
    copts = imp_copts(),
    tags = ["nobuilder"],
    deps = [
        ":async_canvas_source",
        ":canvas_interfaces",
        ":wasm_async_canvas_source",
        "//:api",
        "//core/async:future",
        "//core/math",
        "//core/render",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

# Note that the imp_app rule adds the `--bind` and `-s ALLOW_MEMORY_GROWTH=1` linkopt flag. If you
# are depending on this library without the imp_app rule, ensure that you have those linkopt flags.
cc_library(
    name = "wasm_async_canvas_source",
    srcs = ["wasm_async_canvas_source.cc"],
    hdrs = ["wasm_async_canvas_source.h"],
    copts = imp_copts(),
    tags = ["nobuilder"],
    deps = [
        ":async_canvas_source",
        ":canvas_interfaces",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/canvas/fonts:font_holder",
        "//core/canvas/fonts:font_params_proto_cc",
        "//core/common:invocable",
        "//core/common:log",
        "//core/common:small_source_location",
        "//core/geometry/shapes",
        "//core/math",
        "//core/render",
        "//core/text:text_helpers",
        "//core/view:base_view",
        "//core/view/platforms/wasm:wasm_canvas_manager",
        "//third_party/emscripten:GL-webgl2-full_es3",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:variant",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)

# Unlike Android, the iOS platform canvas source needs to be in a separate library
# so that it can be built as an objc_library.
objc_library(
    name = "ios_platform_canvas_source",
    srcs = [
        "ios_platform_canvas_source.mm",
    ],
    hdrs = [
        "ios_platform_canvas_source.h",
    ],
    deps = [
        ":canvas_interfaces",
        ":constants",
        "//core/canvas/fonts:ios_font_holder",
        "//core/common:log",
        "//core/geometry/shapes",
        "//core/math",
        "//core/render",
        "//core/view:base_view",
        "//third_party/apple_frameworks:CoreText",
        "//third_party/apple_frameworks:Foundation",
        "//third_party/apple_frameworks:UIKit",
        "@com_google_absl//absl/memory",
    ],
)

# --config=android_all
# --android_ndk_min_sdk_version=28

# --config=ios_x86_64

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

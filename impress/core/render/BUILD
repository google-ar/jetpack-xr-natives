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
load("//build_tools:imp.bzl", "if_android", "if_emscripten", "imp_copts", "imp_defines", "imp_google3_copts", "imp_test_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "render",
    srcs = [
        "image_asset.cc",
        "image_helpers.cc",
        "texture.cc",
        "texture_asset.cc",
        "texture_factory.cc",
        "texture_registry.cc",
    ],
    hdrs = [
        "image_asset.h",
        "image_helpers.h",
        "texture.h",
        "texture_asset.h",
        "texture_factory.h",
        "texture_options.h",
        "texture_registry.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":content_security_level",
        ":safe_filament_texture_builder",
        ":texture_builder",
        "//core:config",
        "//core/assets",
        "//core/async:executor",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:invocable",
        "//core/common:owned_or_borrowed_ptr",
        "//core/common:owned_ptr",
        "//core/common:small_source_location",
        "//core/image",
        "//core/image:decode_image",
        "//core/image:inline_image_contents",
        "//core/math",
        "//core/resources",
        "//core/split_engine:split_engine_serializer",
        "//core/view:base_view",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ] + if_emscripten([
        "//core/image:wasm_decode_image",
        "//core/image:wasm_texture_contents",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "safe_filament_texture_builder",
    srcs = [
        "safe_filament_texture_builder.cc",
    ],
    hdrs = [
        "safe_filament_texture_builder.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "base_renderable_manager",
    hdrs = [
        "base_renderable_manager.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/math",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "base_texture_builder",
    hdrs = [
        "base_texture_builder.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/image",
        "@com_google_absl//absl/strings:string_view",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)

cc_library(
    name = "texture_builder",
    srcs = [
        "texture_builder.cc",
    ],
    hdrs = [
        "texture_builder.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":base_texture_builder",
        ":safe_filament_texture_builder",
        "//core/image",
        "//core/view:base_view",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "content_security_level",
    hdrs = [
        "content_security_level.h",
    ],
    defines = imp_defines(),
    alwayslink = 1,
)

cc_library(
    name = "shader_cache_system",
    srcs = ["shader_cache_system.cc"],
    hdrs = ["shader_cache_system.h"],
    copts = imp_test_copts(),
    linkstatic = True,
    deps = [
        "//core/common",
        "//core/common:platform_helpers",
        "//core/common:string_helpers",
        "//core/ncsb",
        "//core/view:base_view",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament/backend:platform",
    ] + if_android([
        "//core/view/platforms/android/wrappers:activity_context",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "display_color_space",
    srcs = ["display_color_space.cc"],
    hdrs = ["display_color_space.h"],
    defines = imp_defines(),
    deps = ["//core/media:media_color_space"],
    alwayslink = 1,
)

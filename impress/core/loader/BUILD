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

load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@third_party//filament:filament.bzl", "by_backend")
load("//build_tools:imp.bzl", "imp_copts", "imp_defines")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

config_setting(
    name = "loader_use_cgltf",
    define_values = {
        "LOADER_USE_CGLTF": "1",
    },
)

config_setting(
    name = "loader_use_tinygltf",
    define_values = {
        "LOADER_USE_TINYGLTF": "1",
    },
)

cc_library(
    name = "loader_config",
    hdrs = ["loader_config.h"],
    copts = imp_copts(),
    defines = by_backend(
        metal = ["LOADER_API_CONFIG=METAL"],
        opengl = ["LOADER_API_CONFIG=OPENGL"],
        vulkan = ["LOADER_API_CONFIG=VULKAN"],
    ),
    linkstatic = True,
    alwayslink = 1,
)

cc_library(
    name = "loader_options",
    hdrs = ["loader_options.h"],
    copts = imp_copts(),
    linkstatic = True,
    alwayslink = 1,
)

cc_library(
    name = "animation_resources",
    srcs = [
        "details/animation_resources.cc",
    ],
    hdrs = [
        "details/animation_resources.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        "//core/animation:fbs",
        "//core/animation:gltf_animation",
        "//core/common",
        "//core/common:flatbuffer_helpers",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "loader_details",
    srcs = [
        "details/bundle_resource_helpers.cc",
    ],
    hdrs = [
        "details/bundle_resource_helpers.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        "//core/common",
        "//core/common:context",
        "//core/common:fbs",
        "//core/common:typed_ids",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # buildcleaner: keep
        "//core/math:flatbuffer_support",
        "@robin_map",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "loaded_model_fb",
    srcs = [
        "details/loaded_model_fb.cc",
    ],
    hdrs = [
        "details/loaded_model_fb.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":animation_resources",
        "//core/common:buffer_access",
        "//core/common:flatbuffer_helpers",
        "//core/common:optional_error",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "loader_creator",
    hdrs = ["loader_creator.h"],
    deps = [
        ":loader",
        ":loader_options",
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:invocable",
        "//core/material_library:material_package",
        "//core/view:base_view",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

# This is an Android-only library for doing sandboxed loading of GLTFs.
# This enables loading of untrusted GLTF models in a safe environment.
cc_library(
    name = "loader_in_sandbox",
    srcs = ["loader_in_sandbox.cc"],
    hdrs = ["loader_in_sandbox.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":loader",
        ":loader_creator",
        ":loader_options",
        "//core/animation:gltf_animation",
        "//core/async:executor",
        "//core/async:future",
        "//core/common",
        "//core/common:flatbuffer_helpers",
        "//core/loader/creator",
        "//core/loader/data:embedded_placeholder_textures",
        "//core/loader/ipc:client",
        "//core/loader/ipc:imp_loader_service_jni",
        "//core/loader/ipc:java_loader_client_jni",
        "//core/loader/ipc:loader_client_base",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/loader/provider/extensions:verification",
        "//core/material_library:material_package",
        "//core/model",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "loader",
    srcs = ["loader.cc"],
    hdrs = [
        "loader.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":loader_options",
        "//core/animation:gltf_animation",
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/model",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "loader_in_process_embedded",
    srcs = [
        "loader_in_process_embedded.cc",
    ],
    hdrs = [
        "loader_in_process_embedded.h",
    ],
    copts = imp_copts(),
    deps = [
        ":loader",
        ":loader_in_process",
        ":loader_options",
        "//core/async:future",
        "//core/common",
        "//core/loader/data:embedded_imp_default_gltf_materials",
        "//core/material_library:material_package",
        "//core/resources",
        "//core/view:base_view",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:string_view",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "loader_in_process",
    srcs = [
        "loader_in_process.cc",
    ],
    hdrs = [
        "loader_in_process.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":loader",
        ":loader_options",
        "//core/animation:gltf_animation",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:flatbuffer_helpers",
        "//core/loader/creator",
        "//core/loader/data:embedded_placeholder_textures",
        "//core/loader/provider",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/loader/provider/extensions:verification",
        "//core/material_library:material_package",
        "//core/model",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_binary(
    name = "loader_tool",
    srcs = ["loader_tool.cc"],
    copts = imp_copts(),
    deps = [
        ":loader_options",
        "//core/common",
        "//core/common:fbs",
        "//core/common:flatbuffer_helpers",
        "//core/loader/data:embedded_imp_default_gltf_materials",
        "//core/loader/data:embedded_placeholder_textures",
        "//core/loader/provider",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/loader/provider/extensions:all_impl",
        "//core/loader/provider/extensions:verification",
        "//core/loader/provider/usdz",
        "//core/material_library:fbs",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament",
    ],
)

GENERIC_MATERIALS_DIR = "third_party/impress/core/loader/data"

GENERIC_MATERIALS_RUNFILES_DIR_DEFINE = "GENERIC_MATERIALS_RUNFILES_DIR=\\\"/google3/%s\\\"" % GENERIC_MATERIALS_DIR

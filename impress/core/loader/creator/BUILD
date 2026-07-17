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
load("//build_tools:imp.bzl", "imp_copts", "imp_defines")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "inflight_creation",
    srcs = [
        "inflight_creation.cc",
    ],
    hdrs = [
        "inflight_creation.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        "//core/common:invocable",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "model_creator_helper",
    srcs = [
        "model_creator_helper.cc",
    ],
    hdrs = [
        "model_creator_helper.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/math",
        "//core/model/mesh:mesh_data",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "model_creator",
    srcs = [
        "model_creator.cc",
    ],
    hdrs = [
        "model_creator.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":create_mesh_data",
        ":inflight_creation",
        ":resource_builders",
        "//core/async:future",
        "//core/common",
        "//core/common:bit_vector",
        "//core/common:data_helpers",
        "//core/common:fbs",
        "//core/common:flatbuffer_helpers",
        "//core/common:hash",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/common:trace",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/image",
        "//core/loader:loader_options",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/loader/provider/extensions:extension_interactivity",
        "//core/loader/provider/extensions:verification",
        "//core/loader/provider/extensions/interactivity:model_creator_extension",
        "//core/material_library:fbs",
        "//core/material_library:generic_material",
        "//core/material_library:generic_material_parameters",
        "//core/material_library:generic_material_spec",
        "//core/material_library:material_package",
        "//core/material_library:material_param_value",
        "//core/math",
        "//core/math:flatbuffer_support",
        "//core/model",
        "//core/model:model_data_details",
        "//core/model/mesh:base_mesh_builder",
        "//core/model/mesh:mesh_builder",
        "//core/render",
        "//core/view:base_view",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "load_image",
    srcs = [
        "load_image.cc",
    ],
    hdrs = [
        "load_image.h",
    ],
    copts = imp_copts(),
    deps = [
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/image",
        "//core/image:decode_image",
        "//core/image:wrapped_image_contents",
        "//core/loader:loaded_model_fb",  # buildcleaner: keep
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/material_library:fbs",
        "//core/render",
        "//core/resources",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/types:span",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:ktxreader_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "resource_builders",
    srcs = [
        "resource_builders.cc",
    ],
    hdrs = [
        "resource_builders.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":inflight_creation",
        "//core/common",
        "//core/common:fbs",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/image",
        "//core/loader:loader_details",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/material_library:fbs",
        "//core/material_library:fbs_deps",  # build_cleaner: keep
        "//core/math",
        "//core/model/mesh:base_mesh_builder",
        "//core/model/mesh:mesh_builder",
        "//core/render",
        "//core/view:base_view",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "creator",
    srcs = [
        "creator.cc",
    ],
    hdrs = [
        "creator.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":load_image",
        ":model_creator",
        "//core/animation:fbs",
        "//core/animation:gltf_animation",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:filament_engine_helpers",
        "//core/common:flatbuffer_helpers",
        "//core/common:trace",
        "//core/image",
        "//core/loader:animation_resources",
        "//core/loader:loader_options",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/material_library:material_package",
        "//core/model",
        "//core/model:model_data_details",
        "//core/render",
        "//core/view:base_view",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "create_mesh_data",
    srcs = ["create_mesh_data.cc"],
    hdrs = ["create_mesh_data.h"],
    deps = [
        ":model_creator_helper",
        "//core/common:fbs",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/loader:loader_details",
        "//core/loader:loader_options",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/model/mesh:mesh_data",
    ],
    alwayslink = 1,
)

cc_library(
    name = "fake_image_contents",
    testonly = True,
    srcs = [
        "fake_image_contents.cc",
    ],
    hdrs = [
        "fake_image_contents.h",
    ],
    copts = imp_copts(),
    deps = [
        "//core/image",
        "//core/loader/data:embedded_imp_default_gltf_materials",
        "//core/loader/data:embedded_placeholder_textures",
        "//testing",
        "@mediapipe//mediapipe/framework/port:gtest_main",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

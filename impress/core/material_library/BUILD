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

load(
    "@flatbuffers//:build_defs.bzl",
    "flatbuffer_cc_library",
)
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:assets.bzl", "imp_assets")
load(
    "//build_tools:imp.bzl",
    "IMP_FLATBUFFER_INCLUDE_PATHS",
    "IMP_FLATC_ARGS",
    "imp_copts",
    "imp_defines",
    "imp_embed_generic_materials",
)
load(
    "//build_tools:imp_gstatic_urls.bzl",
    "imp_gstatic_urls",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

GENERIC_MATERIALS_ZIP = "//core/loader/data:compiled_imp_default_gltf_materials.zip"

GENERIC_LITE_MATERIALS_ZIP = "//core/loader/data:compiled_imp_default_lite_gltf_materials.zip"

imp_assets(
    name = "generic_materials",
    srcs = [GENERIC_MATERIALS_ZIP],
    embed = imp_embed_generic_materials(),
    namespace = "imp::materials",
    urls = imp_gstatic_urls(
        [GENERIC_MATERIALS_ZIP],
        ["compiled_imp_default_gltf_materials"],
    ),
)

imp_assets(
    name = "generic_lite_materials",
    srcs = [GENERIC_LITE_MATERIALS_ZIP],
    embed = imp_embed_generic_materials(),
    namespace = "imp::materials",
    urls = imp_gstatic_urls(
        [GENERIC_LITE_MATERIALS_ZIP],
        ["compiled_imp_default_lite_gltf_materials"],
    ),
)

# Version of generic materials assets that are always embedded regardless of build flags.
# This is used in tests to embed required assets that would normally be downloaded.
# See mock_url_loader.h for how this is used.
imp_assets(
    name = "generic_materials_always_embedded",
    srcs = [GENERIC_MATERIALS_ZIP],
    base_url = "https://www.gstatic.com/ar/core/viewer",
    embed = True,
    namespace = "imp::materials_embedded",
)

imp_assets(
    name = "generic_lite_materials_always_embedded",
    srcs = [GENERIC_LITE_MATERIALS_ZIP],
    base_url = "https://www.gstatic.com/ar/core/viewer",
    embed = True,
    namespace = "imp::materials_embedded",
)

schemas = [
    "schemas/generic_material.fbs",
]

filegroup(
    name = "schemas",
    srcs = schemas,
)

flatbuffer_cc_library(
    name = "fbs",
    srcs = schemas,
    flatc_args = IMP_FLATC_ARGS,
    include_paths = IMP_FLATBUFFER_INCLUDE_PATHS,
    includes = [
        "//core/common:schemas",
    ],
    out_prefix = "schemas/",
)

cc_library(
    name = "fbs_deps",
    deps = [
        ":fbs",
        "//core/common:fbs_deps",
    ],
    alwayslink = 1,
)

cc_library(
    name = "generic_material",
    srcs = [
        "generic_material_impl.cc",
    ],
    hdrs = [
        "generic_material.h",
        "generic_material_impl.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":generic_material_constants",
        ":generic_material_parameters",
        ":generic_material_spec",
        ":material_package",
        ":material_param_value",
        "//core:config",
        "//core/async:future",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/common:typed_ids",
        "//core/materials",
        "//core/math",
        "//core/model:model_data_details",
        "//core/render",
        "//core/split_engine:split_engine_serializer",
        "//core/view:base_view",
        "//core/view/utils:string_map",
        "//core/view/utils:string_set",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/functional:function_ref",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "generic_material_constants",
    hdrs = [
        "generic_material_constants.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        "//core/math",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "generic_material_parameters",
    srcs = ["generic_material_parameters.cc"],
    hdrs = ["generic_material_parameters.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":fbs",
        ":fbs_deps",  # buildcleaner: keep
        ":flatbuffer_utils",
        ":generic_material_constants",
        "//core/common:fbs",
        "//core/math",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "generic_material_schema_provider",
    srcs = ["generic_material_schema_provider.cc"],
    hdrs = ["generic_material_schema_provider.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":fbs",
        ":fbs_deps",  # buildcleaner: keep
        ":generic_material_constants",
        ":generic_material_parameters",
        ":generic_material_spec",
        ":material_param_value",
        "//core/common:fbs",
        "//core/loader/provider:fbs",
        "//core/loader/provider:loaded_model_builder",
        "//core/loader/provider/gltf",
        "//core/loader/provider/gltf:gltf_lookup",
        "//core/loader/provider/gltf:imp_cc_schema",
        "//core/math",
        "//core/math:flatbuffer_support",
        "//core/model:model_data_details",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "generic_material_spec",
    srcs = [
        "generic_material_spec.cc",
    ],
    hdrs = [
        "generic_material_spec.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":fbs",
        ":fbs_deps",  # buildcleaner: keep
        "@com_google_absl//absl/strings:str_format",
        "@flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "material_package",
    srcs = ["material_package.cc"],
    hdrs = ["material_package.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":fbs",
        ":fbs_deps",  # buildcleaner: keep
        ":generic_material_spec",
        "//core:config",
        "//core/assets/material:material_helpers",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/common:zip_helpers",
        "//core/resources",
        "//core/view:base_view",
        "//core/view/platforms/xr_android:xr_color_space_helper",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "material_param_value",
    hdrs = ["material_param_value.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/math",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "flatbuffer_utils",
    srcs = ["flatbuffer_utils.cc"],
    hdrs = ["flatbuffer_utils.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":fbs",
        ":fbs_deps",  # buildcleaner: keep
        "//core/common:type_helpers",
        "//core/loader/provider/gltf:imp_cc_schema",
        "//core/math",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "test_helpers",
    testonly = True,
    srcs = ["test_helpers.cc"],
    hdrs = ["test_helpers.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "@mediapipe//mediapipe/framework/port:gtest",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

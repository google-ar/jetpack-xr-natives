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
load("//build_tools:assets.bzl", "imp_assets")
load(
    "//build_tools:imp.bzl",
    "imp_defines",
    "imp_google3_copts",
)
load("//build_tools:imp_material.bzl", "imp_material")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_assets(
    name = "placeholder_material_asset",
    srcs = [
        ":placeholder_material",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "placeholder_material",
    src = ":placeholder.mat",
)

cc_library(
    name = "split_engine_generic_material",
    srcs = ["split_engine_generic_material.cc"],
    hdrs = ["split_engine_generic_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_texture_parameter_creator",
        ":split_engine_material",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/common:typed_ids",
        "//core/material_library:fbs",
        "//core/material_library:generic_material",
        "//core/material_library:generic_material_constants",
        "//core/material_library:generic_material_parameters",
        "//core/material_library:generic_material_spec",
        "//core/material_library:material_param_value",
        "//core/materials",
        "//core/math",
        "//core/model:model_data_details",
        "//core/render",
        "//core/split_engine:split_engine_serializer",
        "//core/split_engine/android:split_engine_android_bridge",
        "//core/split_engine/materials/builtin:builtin_generic_material",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:string_map",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

config_setting(
    name = "imp_use_local_split_engine_materials",
    define_values = {"IMP_USE_LOCAL_SPLIT_ENGINE_MATERIALS": "1"},
)

cc_library(
    name = "split_engine_material",
    srcs = ["split_engine_material.cc"],
    hdrs = ["split_engine_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_texture_parameter_creator",
        ":placeholder_material_asset",
        ":split_engine_material_factory",
        "//core/assets",
        "//core/async:future",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_serializer",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android:split_engine_android_bridge",
        "//core/split_engine/materials/builtin:builtin_material",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_material_factory",
    srcs = ["split_engine_material_factory.cc"],
    hdrs = ["split_engine_material_factory.h"],
    copts = imp_google3_copts(),
    deps = [
        ":builtin_material_creator_helper",
        "//core/async:future",
        "//core/material_library:generic_material_spec",
        "//core/material_library:material_package",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/materials/builtin:builtin_generic_material",
        "//core/split_engine/materials/builtin:builtin_jxr_media_material",
        "//core/split_engine/materials/builtin:builtin_material",
        "//core/split_engine/materials/builtin:builtin_svxr_footprint_material",
        "//core/split_engine/materials/builtin:builtin_svxr_plane_material",
        "//core/split_engine/materials/builtin:builtin_texture_external_material",
        "//core/split_engine/materials/builtin:builtin_vignette_material",
        "//core/split_engine/materials/builtin:builtin_water_material",
        "//core/split_engine/materials/builtin:builtin_youtube_stereo_player_material",
        "//core/split_engine/materials/builtin/photosxr:builtin_photos_texture_3d_material",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:generic_materials",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "builtin_texture_parameter_creator",
    srcs = ["builtin_texture_parameter_creator.cc"],
    hdrs = ["builtin_texture_parameter_creator.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/common:small_source_location",
        "//core/material_library:flatbuffer_utils",
        "//core/render",
        "//core/split_engine:split_engine_serializer",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "builtin_material_creator_helper",
    srcs = ["builtin_material_creator_helper.cc"],
    hdrs = ["builtin_material_creator_helper.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/async:future",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
        "@third_party//split_engine:fbs_with_embedded_schema",
    ],
    alwayslink = 1,
)

# Copyright 2025 Google LLC
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
    "@com_google_impress//build_tools:imp.bzl",
    "imp_google3_copts",
)
load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "jxr_media_material",
    srcs = ["jxr_media_material.cc"],
    hdrs = ["jxr_media_material.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/async:future",
        "@com_google_impress//core/common:log",
        "@com_google_impress//core/common:type_helpers",
        "@com_google_impress//core/materials",
        "@com_google_impress//core/math",
        "@com_google_impress//core/media:media_color_space",
        "@com_google_impress//core/media:media_type",
        "@com_google_impress//core/render",
        "@com_google_impress//core/split_engine:fbs",
        "@com_google_impress//core/split_engine:fbs_deps",  # buildcleaner: keep
        "@com_google_impress//core/split_engine:flatbuffer_utils",
        "@com_google_impress//core/split_engine:split_engine_serializer",
        "@com_google_impress//core/split_engine/materials:builtin_texture_parameter_creator",
        "@com_google_impress//core/split_engine/materials:split_engine_builtin_material",
        "@com_google_impress//core/view:base_view",
        "@flatbuffers",
        "@third_party//split_engine:split_engine_schema_version",
    ],
    alwayslink = 1,
)

cc_library(
    name = "photos_texture_3d_material_params",
    hdrs = ["photos_texture_3d_material_params.h"],
    deps = [
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//:api",
        "@com_google_impress//core/common:log",
        "@com_google_impress//core/math",
        "@com_google_impress//core/render",
        "@com_google_impress//core/split_engine:flatbuffer_utils",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "photos_texture_3d_material",
    srcs = ["photos_texture_3d_material.cc"],
    hdrs = ["photos_texture_3d_material.h"],
    copts = imp_google3_copts(),
    deps = [
        ":photos_texture_3d_material_params",
        "@com_google_absl//absl/memory",
        "@com_google_impress//:api",
        "@com_google_impress//core/async:future",
        "@com_google_impress//core/materials",
        "@com_google_impress//core/split_engine/materials:builtin_texture_parameter_creator",
        "@com_google_impress//core/split_engine/materials:split_engine_builtin_material",
        "@com_google_impress//core/view:base_view",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "svxr_footprint_material",
    srcs = ["svxr_footprint_material.cc"],
    hdrs = ["svxr_footprint_material.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/memory",
        "@com_google_impress//:api",
        "@com_google_impress//core/async:future",
        "@com_google_impress//core/materials",
        "@com_google_impress//core/math",
        "@com_google_impress//core/split_engine:flatbuffer_utils",
        "@com_google_impress//core/split_engine/materials:builtin_texture_parameter_creator",
        "@com_google_impress//core/split_engine/materials:split_engine_builtin_material",
        "@com_google_impress//core/view:base_view",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "svxr_plane_material",
    srcs = ["svxr_plane_material.cc"],
    hdrs = ["svxr_plane_material.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/memory",
        "@com_google_impress//:api",
        "@com_google_impress//core/async:future",
        "@com_google_impress//core/materials",
        "@com_google_impress//core/math",
        "@com_google_impress//core/split_engine:flatbuffer_utils",
        "@com_google_impress//core/split_engine/materials:builtin_texture_parameter_creator",
        "@com_google_impress//core/split_engine/materials:split_engine_builtin_material",
        "@com_google_impress//core/view:base_view",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "vignette_material",
    srcs = ["vignette_material.cc"],
    hdrs = ["vignette_material.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/memory",
        "@com_google_impress//:api",
        "@com_google_impress//core/async:future",
        "@com_google_impress//core/materials",
        "@com_google_impress//core/math",
        "@com_google_impress//core/split_engine:flatbuffer_utils",
        "@com_google_impress//core/split_engine/materials:builtin_texture_parameter_creator",
        "@com_google_impress//core/split_engine/materials:split_engine_builtin_material",
        "@com_google_impress//core/view:base_view",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "water_reflection_material",
    srcs = ["water_reflection_material.cc"],
    hdrs = ["water_reflection_material.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/memory",
        "@com_google_impress//:api",
        "@com_google_impress//core/async:future",
        "@com_google_impress//core/materials",
        "@com_google_impress//core/math",
        "@com_google_impress//core/render",
        "@com_google_impress//core/split_engine:flatbuffer_utils",
        "@com_google_impress//core/split_engine/materials:builtin_texture_parameter_creator",
        "@com_google_impress//core/split_engine/materials:split_engine_builtin_material",
        "@com_google_impress//core/view:base_view",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "youtube_stereo_player_material",
    srcs = ["youtube_stereo_player_material.cc"],
    hdrs = ["youtube_stereo_player_material.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/memory",
        "@com_google_impress//:api",
        "@com_google_impress//core/async:future",
        "@com_google_impress//core/materials",
        "@com_google_impress//core/render",
        "@com_google_impress//core/split_engine/materials:builtin_texture_parameter_creator",
        "@com_google_impress//core/split_engine/materials:split_engine_builtin_material",
        "@com_google_impress//core/view:base_view",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "texture_external_material",
    srcs = ["texture_external_material.cc"],
    hdrs = ["texture_external_material.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/memory",
        "@com_google_impress//core/async:future",
        "@com_google_impress//core/materials",
        "@com_google_impress//core/render",
        "@com_google_impress//core/split_engine/materials:builtin_texture_parameter_creator",
        "@com_google_impress//core/split_engine/materials:split_engine_builtin_material",
        "@com_google_impress//core/view:base_view",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "gsplat_material_serializer",
    srcs = ["gsplat_material_serializer.cc"],
    hdrs = ["gsplat_material_serializer.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/assets",
        "@com_google_impress//core/async:future",
        "@com_google_impress//core/common:log",
        "@com_google_impress//core/gsplat:gsplat_asset",
        "@com_google_impress//core/gsplat:gsplat_material_params",
        "@com_google_impress//core/materials",
        "@com_google_impress//core/math",
        "@com_google_impress//core/ncsb",
        "@com_google_impress//core/render",
        "@com_google_impress//core/render_passes:texture_pipeline_renderer_projection_quad",
        "@com_google_impress//core/render_passes/group_to_projection_quad_texture_renderer:group_to_projection_quad_texture_renderer_state_proto_cc_imp",
        "@com_google_impress//core/split_engine:flatbuffer_utils",
        "@com_google_impress//core/split_engine:split_engine_serializer",
        "@com_google_impress//core/split_engine/materials:builtin_texture_parameter_creator",
        "@com_google_impress//core/split_engine/materials:split_engine_builtin_material",
        "@com_google_impress//core/view:base_view",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

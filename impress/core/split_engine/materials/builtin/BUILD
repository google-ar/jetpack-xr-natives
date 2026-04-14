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

cc_library(
    name = "builtin_material",
    hdrs = ["builtin_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/common:small_source_location",
        "//core/materials",
        "//core/render",
        "//core/render:display_color_space",
        "@com_google_absl//absl/status",
        "@flatbuffers",
        "@third_party//filament",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "builtin_material_wrapper",
    hdrs = ["builtin_material_wrapper.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_material",
        "//core/common:owned_ptr",
        "//core/common:small_source_location",
        "//core/materials",
        "//core/math",
        "//core/render",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/functional:function_ref",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@third_party//filament",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "builtin_custom_material",
    srcs = ["builtin_custom_material.cc"],
    hdrs = ["builtin_custom_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_material_wrapper",
        "//core:config",
        "//core/common:registry",
        "//core/materials",
        "//core/math",
        "//core/media:media_color_space",
        "//core/render:display_color_space",
        "//core/split_engine:split_engine_external_texture_color_space_store",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "builtin_generic_spec_helpers",
    srcs = ["builtin_generic_spec_helpers.cc"],
    hdrs = ["builtin_generic_spec_helpers.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/material_library:fbs",
        "//core/material_library:fbs_deps",  # buildcleaner: keep
        "//core/material_library:generic_material_spec",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "builtin_generic_material",
    srcs = ["builtin_generic_material.cc"],
    hdrs = ["builtin_generic_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_generic_spec_helpers",
        ":builtin_material",
        ":builtin_material_registry",
        ":builtin_material_wrapper",
        "//core/async:future",
        "//core/material_library:fbs_deps",  # buildcleaner: keep
        "//core/material_library:generic_material",
        "//core/material_library:generic_material_parameters",
        "//core/material_library:generic_material_spec",
        "//core/material_library:material_package",
        "//core/material_library:material_param_value",
        "//core/render",
        "//core/render:display_color_space",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@flatbuffers",
        "@third_party//filament",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,  # Required for the built-in material registry.
)

cc_library(
    name = "builtin_jxr_media_material",
    srcs = ["builtin_jxr_media_material.cc"],
    hdrs = ["builtin_jxr_media_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_custom_material",
        ":builtin_jxr_media_material_assets",
        ":builtin_material",
        ":builtin_material_registry",
        ":jxr_media_material_utils",
        "//core:config",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/material_library:flatbuffer_utils",
        "//core/material_library:material_package",
        "//core/materials",
        "//core/math",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/ncsb",
        "//core/render",
        "//core/render:display_color_space",
        "//core/resources",
        "//core/split_engine:fbs",
        "//core/split_engine:fbs_deps",  # buildcleaner: keep
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,  # Required for the built-in material registry.
)

imp_assets(
    name = "builtin_jxr_media_material_assets",
    srcs = [
        ":builtin_jxr_media_mat",
        ":builtin_jxr_media_opaque_mat",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "builtin_jxr_media_mat",
    src = "builtin_jxr_media.mat",
    includes = [
        "//core/materials:color_conversion_helpers.glsl",
        "//core/materials:super_sample.glsl",
        "//core/xr/media/data:dataspace_support.glsl",
        "//core/xr/media/data:stereo.glsl",
    ],
    replacements = {
        "{BLENDING_MODE}": "transparent",
        "{DEPTH_CULLING}": "false",
        "{IS_OPAQUE}": "false",
    },
)

imp_material(
    name = "builtin_jxr_media_opaque_mat",
    src = "builtin_jxr_media.mat",
    includes = [
        "//core/materials:color_conversion_helpers.glsl",
        "//core/materials:super_sample.glsl",
        "//core/xr/media/data:dataspace_support.glsl",
        "//core/xr/media/data:stereo.glsl",
    ],
    replacements = {
        "{BLENDING_MODE}": "opaque",
        "{DEPTH_CULLING}": "true",
        "{IS_OPAQUE}": "true",
    },
)

cc_library(
    name = "jxr_media_material_utils",
    srcs = ["jxr_media_material_utils.cc"],
    hdrs = ["jxr_media_material_utils.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/math",
        "//core/media:media_type",
    ],
    alwayslink = 1,
)

cc_library(
    name = "builtin_svxr_footprint_material",
    srcs = ["builtin_svxr_footprint_material.cc"],
    hdrs = ["builtin_svxr_footprint_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_custom_material",
        ":builtin_material",
        ":builtin_material_registry",
        ":builtin_svxr_footprint_material_assets",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:future",
        "//core/material_library:material_package",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,  # Required for the built-in material registry.
)

imp_assets(
    name = "builtin_svxr_footprint_material_assets",
    srcs = [
        ":builtin_svxr_footprint_mat",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "builtin_svxr_footprint_mat",
    src = ":builtin_svxr_footprint.mat",
    includes = [
        "//core/materials:color_conversion_helpers.glsl",
    ],
)

cc_library(
    name = "builtin_svxr_plane_material",
    srcs = ["builtin_svxr_plane_material.cc"],
    hdrs = ["builtin_svxr_plane_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_custom_material",
        ":builtin_material",
        ":builtin_material_registry",
        ":builtin_svxr_plane_material_assets",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/material_library:flatbuffer_utils",
        "//core/material_library:material_package",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,  # Required for the built-in material registry.
)

imp_assets(
    name = "builtin_svxr_plane_material_assets",
    srcs = [
        ":builtin_svxr_plane_mat",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "builtin_svxr_plane_mat",
    src = ":builtin_svxr_plane.mat",
    includes = [
        "//core/materials:color_conversion_helpers.glsl",
    ],
)

cc_library(
    name = "builtin_vignette_material",
    srcs = ["builtin_vignette_material.cc"],
    hdrs = ["builtin_vignette_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_custom_material",
        ":builtin_material",
        ":builtin_material_registry",
        ":builtin_vignette_material_assets",
        "//core/assets",
        "//core/async:future",
        "//core/material_library:material_package",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,  # Required for the built-in material registry.
)

imp_assets(
    name = "builtin_texture_external_material_assets",
    srcs = [
        ":builtin_texture_external_mat",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "builtin_texture_external_mat",
    src = ":builtin_texture_external.mat",
    includes = [
        "//core/materials:color_conversion_helpers.glsl",
    ],
)

cc_library(
    name = "builtin_texture_external_material",
    srcs = ["builtin_texture_external_material.cc"],
    hdrs = ["builtin_texture_external_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_custom_material",
        ":builtin_material",
        ":builtin_material_registry",
        ":builtin_texture_external_material_assets",
        "//core/assets",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/material_library:flatbuffer_utils",
        "//core/material_library:material_package",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,  # Required for the built-in material registry.
)

imp_assets(
    name = "builtin_vignette_material_assets",
    srcs = [
        ":builtin_vignette_mat",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "builtin_vignette_mat",
    src = ":builtin_vignette.mat",
    includes = [
        "//core/materials:color_conversion_helpers.glsl",
    ],
)

cc_library(
    name = "builtin_water_material",
    srcs = ["builtin_water_material.cc"],
    hdrs = ["builtin_water_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_custom_material",
        ":builtin_material",
        ":builtin_material_registry",
        ":builtin_water_material_assets",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/material_library:flatbuffer_utils",
        "//core/material_library:material_package",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,  # Required for the built-in material registry.
)

imp_assets(
    name = "builtin_water_material_assets",
    srcs = [
        ":builtin_water_mat",
        ":builtin_water_transparent_mat",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "builtin_water_mat",
    src = ":builtin_water.mat",
    includes = [
        "//core/materials:color_conversion_helpers.glsl",
    ],
)

imp_material(
    name = "builtin_water_transparent_mat",
    src = ":builtin_water.transparent.mat",
    includes = [
        "//core/materials:color_conversion_helpers.glsl",
    ],
)

cc_library(
    name = "builtin_youtube_stereo_player_material",
    srcs = ["builtin_youtube_stereo_player_material.cc"],
    hdrs = ["builtin_youtube_stereo_player_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_custom_material",
        ":builtin_material",
        ":builtin_material_registry",
        ":builtin_youtube_stereo_player_material_assets",
        "//core/assets",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/material_library:flatbuffer_utils",
        "//core/material_library:material_package",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/render:display_color_space",
        "//core/render/android:platform_android_external_texture_surface",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,  # Required for the built-in material registry.
)

imp_assets(
    name = "builtin_youtube_stereo_player_material_assets",
    srcs = [
        ":builtin_youtube_stereo_player_mat",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "builtin_youtube_stereo_player_mat",
    src = ":builtin_youtube_stereo_player.mat",
    includes = [
        "//core/materials:color_conversion_helpers.glsl",
        "//core/materials:super_sample.glsl",
        "//core/xr/media/data:dataspace_support.glsl",
    ],
)

cc_library(
    name = "builtin_material_registry",
    srcs = ["builtin_material_registry.cc"],
    hdrs = ["builtin_material_registry.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_material",
        "//core/async:future",
        "//core/common:invocable",
        "//core/common:small_source_location",
        "//core/material_library:material_package",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "@com_google_absl//absl/base:no_destructor",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

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

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:assets.bzl", "imp_assets")
load(
    "//build_tools:imp.bzl",
    "imp_defines",
    "imp_google3_copts",
)
load("//build_tools:imp_material.bzl", "imp_material")

package(default_visibility = ["//visibility:public"])

exports_files(
    ["gsplat_common.glsl"],
)

# Provides a builtin material type supporting gsplats in split engine.
cc_library(
    name = "precompute_texture_pipeline",
    srcs = ["precompute_texture_pipeline.cc"],
    hdrs = ["precompute_texture_pipeline.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":gsplat_material_deserializer_assets",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/geometry/shapes",
        "//core/materials",
        "//core/math",
        "//core/model/mesh",
        "//core/ncsb",
        "//core/render",
        "//core/render:mesh_renderer",
        "//core/render_passes:texture_pipeline_renderer",
        "//core/render_passes:texture_pipeline_renderer_proto_cc_imp",
        "//core/resources",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:render_settings_proto_cc_imp",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

# Provides a builtin material type supporting gsplats in split engine.
cc_library(
    name = "gsplat_material_deserializer",
    srcs = ["gsplat_material_deserializer.cc"],
    hdrs = ["gsplat_material_deserializer.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":gsplat_material_deserializer_assets",
        ":precompute_texture_pipeline",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:future",
        "//core/common:log",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/common:small_source_location",
        "//core/material_library:flatbuffer_utils",
        "//core/material_library:material_package",
        "//core/materials",
        "//core/math",
        "//core/ncsb",
        "//core/render",
        "//core/resources",
        "//core/split_engine:fbs",
        "//core/split_engine:fbs_deps",  # buildcleaner: keep
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/materials/builtin:builtin_custom_material",
        "//core/split_engine/materials/builtin:builtin_material",
        "//core/split_engine/materials/builtin:builtin_material_registry",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/window:filament_host",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,  # Required for registration.
)

cc_library(
    name = "gsplat_background_material_deserializer",
    srcs = ["gsplat_background_material_deserializer.cc"],
    hdrs = ["gsplat_background_material_deserializer.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":gsplat_material_deserializer_assets",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:future",
        "//core/material_library:material_package",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine:fbs",
        "//core/split_engine:fbs_deps",  # buildcleaner: keep
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/materials/builtin:builtin_custom_material",
        "//core/split_engine/materials/builtin:builtin_material",
        "//core/split_engine/materials/builtin:builtin_material_registry",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,  # Required for registration.
)

imp_assets(
    name = "gsplat_material_deserializer_assets",
    srcs = [
        ":builtin_gsplat_background_mat",
        ":builtin_gsplat_data_precompute_mat",
        ":builtin_gsplat_mono_mat",
        ":builtin_gsplat_stereo_mat",
        ":builtin_magic_window_mono_mat",
        ":builtin_magic_window_stereo_mat",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "builtin_gsplat_mono_mat",
    src = ":builtin_gsplat.mat",
    enable_metal_postprocessing = True,
    includes = [
        "gsplat_common.glsl",
    ],
    replacements = {
        "{PRECOMPUTE_SAMPLER}": "sampler2d",
    },
)

imp_material(
    name = "builtin_gsplat_stereo_mat",
    src = ":builtin_gsplat.mat",
    enable_metal_postprocessing = True,
    includes = [
        "gsplat_common.glsl",
    ],
    replacements = {
        "{PRECOMPUTE_SAMPLER}": "sampler2dArray",
    },
)

imp_material(
    name = "builtin_magic_window_mono_mat",
    src = ":builtin_magic_window.mat",
    enable_metal_postprocessing = True,
    includes = [
        "gsplat_common.glsl",
    ],
    replacements = {
        "{PRECOMPUTE_SAMPLER}": "sampler2d",
    },
)

imp_material(
    name = "builtin_magic_window_stereo_mat",
    src = ":builtin_magic_window.mat",
    enable_metal_postprocessing = True,
    includes = [
        "gsplat_common.glsl",
    ],
    replacements = {
        "{PRECOMPUTE_SAMPLER}": "sampler2dArray",
    },
)

imp_material(
    name = "builtin_gsplat_background_mat",
    src = ":builtin_gsplat_background.mat",
    enable_metal_postprocessing = True,
    includes = [
    ],
)

imp_material(
    name = "builtin_gsplat_data_precompute_mat",
    src = ":builtin_gsplat_data_precompute.mat",
    enable_metal_postprocessing = True,
    includes = [
        "gsplat_common.glsl",
    ],
)

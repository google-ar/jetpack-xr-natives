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

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "builtin_photos_texture_3d_material",
    srcs = ["builtin_photos_texture_3d_material.cc"],
    hdrs = ["builtin_photos_texture_3d_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_photos_texture_3d_material_assets",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:future",
        "//core/material_library:flatbuffer_utils",
        "//core/materials",
        "//core/math",
        "//core/ncsb",
        "//core/render",
        "//core/render:display_color_space",
        "//core/render/android:platform_android_external_texture_surface",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/materials/builtin:builtin_custom_material",
        "//core/split_engine/materials/builtin:builtin_material",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/framework",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

imp_assets(
    name = "builtin_photos_texture_3d_material_assets",
    srcs = [
        ":builtin_photos_texture_3d_mat",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "builtin_photos_texture_3d_mat",
    src = ":builtin_photos_texture_3d.mat",
    includes = [
        ":color_space_helpers.glsl",
        ":curved_panel.glsl",
        ":panorama_helpers.glsl",
        ":quad_parallax_helpers.glsl",
        ":sdf_fade.glsl",
        ":spatialize_animation.glsl",
        "//core/materials:color_conversion_helpers.glsl",
        "//core/materials:super_sample.glsl",
        "//core/xr/media/data:dataspace_support.glsl",
    ],
)

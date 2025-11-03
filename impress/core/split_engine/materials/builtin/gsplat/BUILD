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

# Provides a builtin material type supporting gsplats in split engine.
cc_library(
    name = "builtin_gsplat_material",
    srcs = ["builtin_gsplat_material.cc"],
    hdrs = ["builtin_gsplat_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_gsplat_material_assets",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:future",
        "//core/material_library:material_param_value",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine:fbs",
        "//core/split_engine:fbs_deps",  # buildcleaner: keep
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/materials/builtin:builtin_custom_material",
        "//core/split_engine/materials/builtin:builtin_material",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "builtin_gsplat_background_material",
    srcs = ["builtin_gsplat_background_material.cc"],
    hdrs = ["builtin_gsplat_background_material.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":builtin_gsplat_material_assets",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:future",
        "//core/material_library:material_param_value",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine:fbs",
        "//core/split_engine:fbs_deps",  # buildcleaner: keep
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/materials/builtin:builtin_custom_material",
        "//core/split_engine/materials/builtin:builtin_material",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

imp_assets(
    name = "builtin_gsplat_material_assets",
    srcs = [
        ":builtin_gsplat_background_mat",
        ":builtin_gsplat_mat",
    ],
    namespace = "imp::split_engine",
)

imp_material(
    name = "builtin_gsplat_mat",
    src = ":builtin_gsplat.mat",
    enable_metal_postprocessing = True,
    includes = [
        "//core/materials:color_conversion_helpers.glsl",
        "//core/xr/media/data:dataspace_support.glsl",
    ],
)

imp_material(
    name = "builtin_gsplat_background_mat",
    src = ":builtin_gsplat_background.mat",
    enable_metal_postprocessing = True,
    includes = [
    ],
)

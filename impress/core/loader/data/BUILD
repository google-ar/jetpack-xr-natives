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

load("@bazel_skylib//:bzl_library.bzl", "bzl_library")
load("@com_google_sandboxed_api//sandboxed_api/bazel:embed_data.bzl", "sapi_cc_embed_data")
load(
    "//build_tools:imp_gltf_materials_package.bzl",
    "generate_imp_default_gltf_materials_package",
    "generate_imp_default_lite_gltf_materials_package",
    "generate_imp_default_performance_gltf_materials_package",
)

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

generate_imp_default_gltf_materials_package()

generate_imp_default_lite_gltf_materials_package()

# Create a material package that compiles materials for performance. Useful for
# testing environments where speed is critical (for example, Forge on Mac iOS tests).
generate_imp_default_performance_gltf_materials_package()

sapi_cc_embed_data(
    name = "embedded_placeholder_textures",
    srcs = [
        "black_placeholder.png",
        "normal_placeholder.png",
        "white_placeholder.png",
    ],
    flatten = True,
)

exports_files([
    "occlusion_helpers_common.glsl",
    "occlusion_helpers_ar_core.glsl",
    "occlusion_helpers_ar_kit.glsl",
    "generic_material_helpers.glsl",
    "generic_material_unlit.mat.template.glsl",
    "generic_material_lit.mat.template.glsl",
    "depth_clear.mat",
])

bzl_library(
    name = "generic_material_bzl",
    srcs = ["generic_material.bzl"],
    deps = [
        "//build_tools:imp_bzl",
        "@bazel_skylib//rules:copy_file",
        "@third_party//filament:filament_bzl",
    ],
)

filegroup(
    name = "generic_material_templates",
    srcs = [
        "generic_material_lit.mat.template.glsl",
        "generic_material_unlit.mat.template.glsl",
    ],
    data = ["//core/materials:color_conversion_helpers.glsl"],
)

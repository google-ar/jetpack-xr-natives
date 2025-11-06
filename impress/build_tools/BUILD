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

# Impress Framework helps internal devs make performant, high quality AR apps.

load("@bazel_skylib//:bzl_library.bzl", "bzl_library")
load("@bazel_skylib//lib:selects.bzl", "selects")
load(
    "//build_tools:imp.bzl",
    "IMP_DEPENDEES",
    "clean_dep",
)

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

# These dependees explicitly need view into Impress Framework internals.
package_group(
    name = "imp_dependees",
    packages = IMP_DEPENDEES,
)

# Built-in imp assets (i.e. gltf materials) are downloaded by default to keep binary size down.
# This config setting is used to instead embed the assets into the binary, which can be useful
# for testing or for applications that require offline use on first startup.
config_setting(
    name = "imp_embed_assets",
    define_values = {
        "IMP_EMBED_ASSETS": "1",
    },
)

selects.config_setting_group(
    name = "should_embed_generic_materials",
    match_any = [
        clean_dep("//build_tools:imp_embed_assets"),
        clean_dep("//core:linux"),
        clean_dep("//core:macos"),
        clean_dep("//core:windows"),
        clean_dep("//core:emscripten"),
        clean_dep("//core:ios_simulator_linux"),
        clean_dep("//core:ios_simulator_linux_metal"),
    ],
)

bzl_library(
    name = "imp_app_bzl",
    srcs = ["imp_app.bzl"],
    deps = [
        ":imp_bzl",
        "//build_tools/platforms/android:imp_android_app_bzl",
        "//build_tools/platforms/android:imp_app_resources_bzl",
        "//build_tools/platforms/desktop:imp_desktop_app_bzl",
        "//build_tools/platforms/desktop_split_engine:imp_desktop_split_engine_app_bzl",
        "//build_tools/platforms/split_engine:imp_split_engine_app_bzl",
        "//build_tools/platforms/xr:imp_xr_app_bzl",
        "//devtools/build_cleaner/skylark:build_defs_lib",
        "//mobile/build:build_defs",
        "@rules_cc//cc:core_rules",
    ],
)

bzl_library(
    name = "imp_material_bzl",
    srcs = ["imp_material.bzl"],
    deps = ["//core/loader/data:generic_material_bzl"],
)

bzl_library(
    name = "imp_texture_bzl",
    srcs = ["imp_texture.bzl"],
)

bzl_library(
    name = "imp_bzl",
    srcs = ["imp.bzl"],
    deps = ["@third_party//filament:filament_bzl"],
)

bzl_library(
    name = "assets_bzl",
    srcs = ["assets.bzl"],
    deps = ["//core/resources:resources_bzl"],
)

bzl_library(
    name = "generated_gstatic_assets_bzl",
    srcs = ["generated_gstatic_assets.bzl"],
)

bzl_library(
    name = "imp_gstatic_urls_bzl",
    srcs = ["imp_gstatic_urls.bzl"],
    deps = [":generated_gstatic_assets_bzl"],
)

bzl_library(
    name = "path_tools_bzl",
    srcs = ["path_tools.bzl"],
)

bzl_library(
    name = "imp_image_based_lighting_bzl",
    srcs = ["imp_image_based_lighting.bzl"],
    deps = [":generate_ibl_bzl"],
)

bzl_library(
    name = "generate_ibl_bzl",
    srcs = ["generate_ibl.bzl"],
    deps = ["@bazel_skylib//lib:paths"],
)

bzl_library(
    name = "imp_gltf_materials_package_bzl",
    srcs = ["imp_gltf_materials_package.bzl"],
    deps = [
        "//core/loader/data:generic_material_bzl",
        "//core/resources:portable_embed_data_bzl",
        "@rules_pkg//:bzl_srcs",
    ],
)

bzl_library(
    name = "imp_extensions_bzl",
    srcs = ["imp_extensions.bzl"],
    deps = [
        ":imp_bzl",
        "@rules_cc//cc:core_rules",
    ],
)

# Exempts files that aren't referenced by BUILD rules from build_cleaners usage check.
# See (broken link)
filegroup(
    name = "ignored_sources",
    srcs = ["binary_diff/symbol_diff.py"],
    tags = ["ignore_srcs"],
)

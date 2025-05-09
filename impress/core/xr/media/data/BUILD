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

load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp_material.bzl", "imp_material")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

exports_files([
    "stereo.glsl",
    "dataspace_support.glsl",
])

imp_material(
    name = "xr_media_skybox_360",
    src = "xr_media_skybox_360.mat",
    includes = [
        ":stereo.glsl",
        "//core/materials:color_conversion_helpers.glsl",
    ],
)

imp_material(
    name = "xr_media_skybox_180",
    src = "xr_media_skybox_180.mat",
    includes = [
        ":stereo.glsl",
        "//core/materials:color_conversion_helpers.glsl",
    ],
)

imp_material(
    name = "xr_media_plane",
    src = "xr_media_plane.mat",
    includes = [
        ":dataspace_support.glsl",
        ":stereo.glsl",
        "//core/materials:color_conversion_helpers.glsl",
    ],
)

imp_assets(
    name = "assets",
    srcs = [
        ":xr_media_plane",
        ":xr_media_skybox_180",
        ":xr_media_skybox_360",
    ],
    namespace = "imp::xrmedia",
)

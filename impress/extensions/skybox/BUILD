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

# Components for Impress Previewer.
load("//build_tools:imp.bzl", "imp_google3_copts")
load("//build_tools:imp_material.bzl", "imp_material")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_material(
    name = "skybox_material",
    src = ":skybox.mat",
)

imp_assets(
    name = "skybox_assets",
    srcs = [
        ":skybox_material",
    ],
    namespace = "imp",
)

cc_library(
    name = "skybox_renderer",
    srcs = [
        "skybox_renderer.cc",
    ],
    hdrs = [
        "skybox_renderer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":skybox_assets",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:future",
        "//core/lighting:environment_light",
        "//core/ncsb",
        "//core/render",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

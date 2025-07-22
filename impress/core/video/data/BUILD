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
load("//build_tools:imp_material.bzl", "imp_material")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_material(
    name = "video_material",
    src = "video.mat",
)

imp_material(
    name = "video_unlit_material",
    src = "video_unlit.mat",
)

imp_assets(
    name = "video_assets",
    srcs = [
        ":video_material",
        ":video_unlit_material",
    ],
    namespace = "imp::video",
)

cc_library(
    name = "video_assets_cc",
    deps = [
        ":video_assets",
    ],
    alwayslink = 1,
)

# Copyright 2026 Google LLC
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
load("//core/proto:imp_isf_data.bzl", "imp_isf_data")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_material(
    name = "outline_replace",
    src = "materials/outline_replace.mat",
)

imp_material(
    name = "outline_h_blur",
    src = "materials/outline_h_blur.mat",
    includes = ["materials/gaussian_kernel.glsl"],
)

imp_material(
    name = "outline_composite",
    src = "materials/outline_composite.mat",
    includes = ["materials/gaussian_kernel.glsl"],
)

imp_isf_data(
    name = "selection_outline_isf",
    srcs = [
        "selection_outline.textproto",
    ],
    deps = [
        "//core/render:primitive_shape_renderer_state_proto",
        "//core/render_passes:texture_pipeline_renderer_proto",
    ],
)

imp_assets(
    name = "outline_assets",
    srcs = [
        ":outline_composite",
        ":outline_h_blur",
        ":outline_replace",
        ":selection_outline_isf",
    ],
    namespace = "imp::editor::outline_assets",
)

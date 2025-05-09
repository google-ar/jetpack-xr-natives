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

## light widgets for all types of lights.
load("//build_tools:assets.bzl", "imp_assets")
load("//core/proto:imp_isf_data.bzl", "imp_isf_data")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

imp_assets(
    name = "light_visualizer_assets",
    srcs = [
        ":light_isf",
        "//core/editor/visualizers/light_assets:directional_light.gltf",
        "//core/editor/visualizers/light_assets:point_light.gltf",
        "//core/editor/visualizers/light_assets:spot_light.gltf",
    ],
    namespace = "imp::editor::light_visualizer_assets",
)

imp_isf_data(
    name = "light_isf",
    srcs = [
        "light_visualizer.textproto",
    ],
)

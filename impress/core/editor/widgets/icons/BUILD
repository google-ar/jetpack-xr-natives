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

# Icons for the console logger.
load("//build_tools:assets.bzl", "imp_assets")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_assets(
    name = "texture_assets",
    srcs = [
        ":error.png",
        ":file.png",
        ":folder.png",
        ":home.png",
        ":image.png",
        ":info.png",
        ":material.png",
        ":model.png",
        ":pause.png",
        ":pending.png",
        ":play.png",
        ":resume.png",
        ":scene.png",
        ":step.png",
        ":stop.png",
        ":up_arrow.png",
        ":warning.png",
    ],
    namespace = "imp::editor::texture_data",
)

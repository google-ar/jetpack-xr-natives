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

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

imp_assets(
    name = "test_materials",
    srcs = [
        "//apibindings/testing/scuba/materials:debug_bone_indices",
        "//apibindings/testing/scuba/materials:debug_bone_weights",
        "//apibindings/testing/scuba/materials:debug_color",
        "//apibindings/testing/scuba/materials:debug_normal",
        "//apibindings/testing/scuba/materials:debug_uv0",
        "//apibindings/testing/scuba/materials:debug_uv1",
    ],
    namespace = "imp::apibindings::testing::scuba::materials",
)

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
    name = "screen_fader_material",
    src = "screen_fader.mat",
)

imp_assets(
    name = "screen_fader_assets",
    srcs = [
        ":screen_fader_material",
    ],
    namespace = "imp::screen_fader",
)

cc_library(
    name = "screen_fader",
    srcs = [
        "screen_fader.cc",
    ],
    hdrs = [
        "screen_fader.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":screen_fader_assets",
        "//core/async:future",
        "//core/common:smooth",
        "//core/ncsb",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
    ],
    alwayslink = 1,
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

imp_assets(
    name = "test_assets",
    srcs = [
        "//third_party/arcore/googledata/sceneform/assets/gltf_2p0_sample_assets:Avocado_glb",
    ],
    namespace = "test_asssets",
)

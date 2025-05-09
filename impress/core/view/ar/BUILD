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
load("//build_tools:imp.bzl", "if_desktop", "imp_google3_copts")
load("//build_tools:imp_material.bzl", "imp_material")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

#boop
licenses(["notice"])

imp_material(
    name = "camera_material",
    src = "camera_material.mat",
    replacements = {},
)

imp_assets(
    name = "ar_assets",
    srcs = [
        ":camera_material",
    ],
    namespace = "ar_assets",
)

cc_library(
    name = "base_ar_component",
    srcs = [
    ],
    hdrs = [
        "base_ar_component.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        "//core/ar",
        "//core/ar:shared",
        "//core/collision:collider_helpers",
        "//core/collision:collision_flags",
        "//core/common",
        "//core/common:filament_helpers",
        "//core/common:log",
        "//core/ncsb",
        "@com_google_absl//absl/base:core_headers",
        "@robin_map",
    ] + if_desktop([
        "//core/ar/desktop",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "ar",
    srcs = [
        "ar_anchor_component.cc",
        "ar_camera_renderer.cc",
        "ar_scene_controller.cc",
    ],
    hdrs = [
        "ar_anchor_component.h",
        "ar_camera_renderer.h",
        "ar_scene_controller.h",
        "base_ar_component.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":ar_assets",
        "//core/ar",
        "//core/ar:shared",
        "//core/collision:collider_helpers",
        "//core/collision:collision_flags",
        "//core/collision:collision_helpers",
        "//core/collision:plane",
        "//core/common",
        "//core/common:filament_helpers",
        "//core/common:log",
        "//core/common:registry",
        "//core/common:trace",
        "//core/common:tuple_helpers",
        "//core/math",
        "//core/model/mesh",
        "//core/model/mesh:mesh_data",
        "//core/monitor",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/ar/collision",
        "//core/view/framework",
        "//core/window:window_rotation",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:optional",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
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

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
load(
    "//core/proto:imp_cc_proto_library.bzl",
    "imp_cc_proto_library",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_material(
    name = "sprite_material",
    src = ":shaders/sprite.mat",
    enable_metal_postprocessing = True,
    preserve_text_shaders = True,
)

imp_material(
    name = "sprite_external_material",
    src = ":shaders/sprite_external.mat",
    enable_metal_postprocessing = True,
    preserve_text_shaders = True,
)

imp_material(
    name = "sprite_outline_material",
    src = ":shaders/sprite_outline.mat",
    enable_metal_postprocessing = True,
    preserve_text_shaders = True,
)

imp_material(
    name = "sprite_external_outline_material",
    src = ":shaders/sprite_external_outline.mat",
    enable_metal_postprocessing = True,
    preserve_text_shaders = True,
)

imp_assets(
    name = "sprite_renderer_assets",
    srcs = [
        ":sprite_external_material",
        ":sprite_external_outline_material",
        ":sprite_material",
        ":sprite_outline_material",
    ],
    namespace = "imp::sprite_renderer_assets",
)

proto_library(
    name = "sprite_renderer_proto",
    srcs = [
        "sprite_renderer_state.proto",
    ],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
        "//core/view/framework:framework_proto",
    ],
)

imp_cc_proto_library(
    name = "sprite_renderer_imp_cc_proto",
    proto = ":sprite_renderer_proto",
    deps = [
        "//core/math:math_proto_cc",
        "//core/view/framework:framework_proto_cc",
    ],
)

cc_library(
    name = "sprite",
    srcs = ["sprite_renderer.cc"],
    hdrs = ["sprite_renderer.h"],
    copts = imp_google3_copts(),
    deps = [
        ":sprite_renderer_assets",
        ":sprite_renderer_imp_cc_proto",
        "//core/assets",
        "//core/async:future",
        "//core/common:log",
        "//core/geometry/shapes",
        "//core/math",
        "//core/model/mesh",
        "//core/ncsb",
        "//core/render",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/framework",
        "//core/view/utils:asset",
        "//core/view/utils:device",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

imp_material(
    name = "external_material",
    src = ":test_data/external_material.mat",
)

imp_assets(
    name = "test_assets",
    srcs = [
        "test_data/ar_icon.png",
        "test_data/google.png",
        "test_data/pin.png",
        "test_data/ui_pan.png",
        ":external_material",
    ],
    namespace = "imp::test_data",
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

proto_library(
    name = "sprite_collider_proto",
    srcs = [
        "sprite_collider_state.proto",
    ],
)

imp_cc_proto_library(
    name = "sprite_collider_imp_cc_proto",
    proto = ":sprite_collider_proto",
)

cc_library(
    name = "sprite_collider",
    srcs = ["sprite_collider.cc"],
    hdrs = ["sprite_collider.h"],
    copts = imp_google3_copts(),
    deps = [
        ":sprite",
        ":sprite_collider_imp_cc_proto",
        ":sprite_renderer_imp_cc_proto",
        "//core/collision:collider_helpers",
        "//core/collision:collision_flags",
        "//core/collision:collision_helpers",
        "//core/common:debug_draw",
        "//core/common:enum_flags",
        "//core/common:log",
        "//core/common:platform_helpers",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_absl//absl/status",
    ],
    alwayslink = 1,
)

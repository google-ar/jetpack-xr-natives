# Copyright 2025 Google LLC
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
load("//build_tools:imp.bzl", "if_dev_runtime")
load("//core/proto:imp_cc_proto_library.bzl", "imp_cc_proto_library")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_assets(
    name = "test_gsplat_resources",
    srcs = [
        "assets/cinnamon_tuesday_medium.pb",
    ],
    namespace = "imp::test_data",
)

imp_assets(
    name = "gsplat_renderer_scuba_test_resources",
    srcs = [
        "assets/micro_kitchen.ply",
    ],
    namespace = "imp::test_data",
)

proto_library(
    name = "gsplat_proto",
    srcs = ["gsplat.proto"],
)

cc_proto_library(
    name = "gsplat_cc_proto",
    deps = [":gsplat_proto"],
)

cc_library(
    name = "gsplat_asset",
    srcs = ["gsplat_asset.cc"],
    hdrs = ["gsplat_asset.h"],
    deps = [
        ":conversion",
        ":encoding",
        ":gsplat_cc_proto",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/image:borrowed_image_content",
        "//core/image:owned_image_content",
        "//core/math",
        "//core/render",
        "//core/resources",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "gsplat_file_loader",
    srcs = ["gsplat_file_loader.cc"],
    hdrs = ["gsplat_file_loader.h"],
    deps = [
        ":gsplat_asset",
        ":gsplat_renderer",
        "//core/assets",
        "//core/async:future",
        "//core/common:invocable",
        "//core/editor",
        "//core/editor:file_loader_helper",
        "//core/editor:file_type",
        "//core/editor:file_type_loader",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "gsplat_renderer",
    srcs = ["gsplat_renderer.cc"],
    hdrs = ["gsplat_renderer.h"],
    deps = [
        ":encoding",
        ":gsplat_asset",
        ":gsplat_cc_proto",
        ":gsplat_material_params",
        ":gsplat_renderer_state_proto_cc",
        "//core:config",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:executor",
        "//core/async:future",
        "//core/camera",
        "//core/common:owned_ptr",
        "//core/common:small_source_location",
        "//core/common:trace",
        "//core/geometry/shapes",
        "//core/image:borrowed_image_content",
        "//core/image:inline_image_contents",
        "//core/materials",
        "//core/math",
        "//core/model/mesh",
        "//core/model/mesh:mesh_data",
        "//core/model/mesh:mesh_factory",
        "//core/ncsb",
        "//core/render",
        "//core/render:mesh_renderer",
        "//core/render_passes:texture_pipeline_renderer",
        "//core/render_passes/group_to_projection_quad_texture_renderer:group_to_projection_quad_texture_renderer_state_proto_cc_imp",
        "//core/split_engine/materials/builtin/gsplat:gsplat_material_deserializer",
        "//core/split_engine/materials/builtin/gsplat:gsplat_material_helpers",
        "//core/split_engine/materials/builtin/gsplat:precompute_material_assets",
        "//core/split_engine/materials/builtin/gsplat:precompute_texture_pipeline",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "//core/window:filament_host",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//filament/libs:math_hdrs",
    ] + if_dev_runtime([
        "//core/editor:file_type",
        "//core/editor:file_loader_helper",
        "//core/common:registry",
        "//core/editor",
    ]),
    alwayslink = 1,
)

proto_library(
    name = "gsplat_renderer_state_proto",
    srcs = ["gsplat_renderer_state.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
        "//core/scene_handles:scene_handles_proto",
    ],
)

imp_cc_proto_library(
    name = "gsplat_renderer_state_proto_cc",
    proto = ":gsplat_renderer_state_proto",
)

cc_library(
    name = "conversion",
    srcs = ["conversion.cc"],
    hdrs = ["conversion.h"],
    deps = [
        "//core/common:trace",
        "//core/math",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:status",
        "@miniply",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "encoding",
    srcs = ["encoding.cc"],
    hdrs = ["encoding.h"],
    deps = ["//core/math"],
    alwayslink = 1,
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

cc_library(
    name = "gsplat_material_params",
    hdrs = ["gsplat_material_params.h"],
    deps = [
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

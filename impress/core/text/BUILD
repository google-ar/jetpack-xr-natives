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
load("//build_tools:imp.bzl", "if_desktop", "if_dev_runtime", "if_mobile", "imp_google3_copts")
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

exports_files([
    "label_clip_space.glsl",
    "text_colors.glsl",
    "text_material_common_desktop.glsl",
    "text_material_common_mobile.glsl",
])

# TODO: Fix the differences in text material between mobile and desktop.
genrule(
    name = "text_material_common",
    srcs = if_mobile(
        ["//core/text:text_material_common_mobile.glsl"],
        ["//core/text:text_material_common_desktop.glsl"],
    ),
    outs = ["text_material_common.glsl"],
    cmd = "cp $< $@",
)

imp_material(
    name = "text_screen_space_material",
    src = "text_screen_space.mat",
    enable_metal_postprocessing = True,
    includes = [
        ":label_clip_space.glsl",
        ":text_colors.glsl",
        ":text_material_common",
    ],
)

imp_material(
    name = "text_material",
    src = "text.mat",
    enable_metal_postprocessing = True,
    includes = [
        ":text_colors.glsl",
        ":text_material_common",
    ],
)

imp_material(
    name = "text_material_fl0",
    src = "text.mat",
    enable_metal_postprocessing = True,
    force_feature_level_zero = True,
    includes = [
        ":text_colors.glsl",
        ":text_material_common",
    ],
)

imp_material(
    name = "sliced_text_screen_space_material",
    src = "sliced_text_screen_space.mat",
    enable_metal_postprocessing = True,
    includes = [
        ":label_clip_space.glsl",
        ":text_colors.glsl",
        ":text_material_common",
    ],
)

imp_material(
    name = "sliced_text_material",
    src = "sliced_text.mat",
    enable_metal_postprocessing = True,
    includes = [
        ":text_colors.glsl",
        ":text_material_common",
    ],
)

imp_assets(
    name = "text_renderer_assets",
    srcs = [
        ":sliced_text_material",
        ":sliced_text_screen_space_material",
        ":text_material",
        ":text_material_fl0",
        ":text_screen_space_material",
    ],
    namespace = "imp::text_renderer_assets",
)

cc_library(
    name = "glyph_atlas",
    srcs = [
        "glyph_atlas.cc",
    ],
    hdrs = [
        "glyph_atlas.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":glyph_emulator",
        ":text_glyphs",
        ":text_helpers",
        ":text_metrics_cc_proto",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/atlas:atlas_packer",
        "//core/atlas:shelf_atlas_packer",
        "//core/canvas:async_canvas_source",
        "//core/canvas:async_canvas_source_factory",
        "//core/canvas:canvas_interfaces",
        "//core/canvas:constants",
        "//core/canvas/fonts:font_holder",
        "//core/canvas/fonts:font_params_helper",
        "//core/canvas/fonts:font_params_proto_cc",
        "//core/canvas/fonts:system_font_provider",
        "//core/common:hash",
        "//core/common:ref_counter",
        "//core/common:rememberer",
        "//core/common:trace",
        "//core/common:typed_ids",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/render",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/utils:device",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
        "@icu//icu4c/source/common:headers",
    ] + if_dev_runtime([
        "//core/editor",
        "//core/editor:widget_ui_system",
        "//core/editor/layout:editor_panel_ids",
        "//core/text/editor:glyph_atlas_visualizer",
    ]),
    alwayslink = 1,
)

imp_material(
    name = "blit_slice_material",
    src = "blit_slice.mat",
    enable_metal_postprocessing = True,
)

imp_material(
    name = "blit_slice_external_material",
    src = "blit_slice_external.mat",
    enable_metal_postprocessing = True,
)

imp_assets(
    name = "sliced_glyph_atlas_assets",
    srcs = [
        ":blit_slice_external_material",
        ":blit_slice_material",
    ],
    namespace = "imp::sliced_glyph_atlas_assets",
)

cc_library(
    name = "sliced_glyph_atlas",
    srcs = [
        "glyph_atlas_slice.cc",
        "sliced_glyph_atlas.cc",
        "sliced_glyph_atlas_helpers.cc",
        "sliced_glyph_texture_manager.cc",
    ],
    hdrs = [
        "glyph_atlas_slice.h",
        "sliced_glyph_atlas.h",
        "sliced_glyph_atlas_helpers.h",
        "sliced_glyph_texture_manager.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":glyph_emulator",
        ":sliced_glyph_atlas_assets",
        ":text_glyphs",
        ":text_helpers",
        ":text_metrics_cc_proto",
        "//core:config",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/async:executor",
        "//core/async:future",
        "//core/atlas:atlas_packer",
        "//core/atlas:shelf_atlas_packer",
        "//core/canvas:async_canvas_source",
        "//core/canvas:async_canvas_source_factory",
        "//core/canvas:canvas_interfaces",
        "//core/canvas:constants",
        "//core/canvas/fonts:font_holder",
        "//core/canvas/fonts:font_params_helper",
        "//core/canvas/fonts:font_params_proto_cc",
        "//core/canvas/fonts:system_font_provider",
        "//core/common:bit_vector",
        "//core/common:hash",
        "//core/common:ref_counter",
        "//core/common:rememberer",
        "//core/common:small_source_location",
        "//core/common:trace",
        "//core/common:typed_ids",
        "//core/geometry/shapes",
        "//core/materials",
        "//core/math",
        "//core/ncsb",
        "//core/render",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/framework",
        "//core/view/utils:device",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
        "@icu//icu4c/source/common:headers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ] + if_dev_runtime([
        "//core/editor",
        "//core/editor:widget_ui_system",
        "//core/editor/layout:editor_panel_ids",
        "//core/text/editor:sliced_glyph_atlas_visualizer",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "glyph_emulator",
    srcs = ["glyph_emulator.cc"],
    hdrs = ["glyph_emulator.h"],
    copts = imp_google3_copts(),
    deps = [
        ":text_helpers",
        ":text_metrics_cc_proto",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/canvas:async_canvas_source",
        "//core/canvas:canvas_interfaces",
        "//core/canvas:constants",
        "//core/canvas/fonts:font_holder",
        "//core/canvas/fonts:font_params_helper",
        "//core/canvas/fonts:font_params_proto_cc",
        "//core/canvas/fonts:system_font_provider",
        "//core/common:context",
        "//core/common:hash",
        "//core/math",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
        "@icu//icu4c/source/common:headers",
    ],
    alwayslink = 1,
)

proto_library(
    name = "text_renderer_proto",
    srcs = ["text_renderer_state.proto"],
    deps = [
        ":text_metrics_proto",
        "//core/canvas/fonts:font_params_proto",
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "text_renderer_proto_cc",
    proto = ":text_renderer_proto",
    deps = [
        ":text_metrics_cc_proto",
        "//core/math:math_proto_cc",
    ],
)

proto_library(
    name = "text_metrics_proto",
    srcs = ["text_metrics.proto"],
)

cc_proto_library(
    name = "text_metrics_cc_proto",
    deps = [":text_metrics_proto"],
)

cc_library(
    name = "text_glyphs",
    hdrs = ["text_glyphs.h"],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:ref_counter",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/math",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "text_renderer",
    srcs = [
        "text_renderer.cc",
    ],
    hdrs = [
        "text_renderer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":glyph_atlas",
        ":glyph_emulator",
        ":sliced_glyph_atlas",
        ":text_glyphs",
        ":text_metrics_cc_proto",
        ":text_renderer_assets",
        ":text_renderer_proto_cc",
        "//core:config",
        "//core/async:future",
        "//core/canvas:canvas_interfaces",
        "//core/canvas/fonts:font_params_proto_cc",
        "//core/common:allocator_pool",
        "//core/common:filament_helpers",
        "//core/common:invocable",
        "//core/common:registry",
        "//core/geometry:geometry_helper",
        "//core/geometry/shapes",
        "//core/materials",
        "//core/math",
        "//core/model/mesh",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/render",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "//core/view/utils:view_config_proto_cc_imp",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "text_collider",
    srcs = [
        "text_collider.cc",
    ],
    hdrs = [
        "text_collider.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":text_collider_imp_cc_proto",
        ":text_renderer",
        "//core/collision:collider_helpers",
        "//core/collision:collision_flags",
        "//core/collision:collision_helpers",
        "//core/collision:plane",
        "//core/collision:ray",
        "//core/common:debug_draw",
        "//core/common:enum_flags",
        "//core/common:filament_helpers",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_impress//core/common:log",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "text_helpers",
    srcs = ["text_helpers.cc"],
    hdrs = ["text_helpers.h"],
    deps = [
        "//core:config",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@icu//icu4c/source/common:headers",
    ] + if_desktop([
        # icu data heavily inflates binary size so do not include it for non desktop platforms for now
    ]),
    alwayslink = 1,
)

cc_library(
    name = "text_material_helpers",
    srcs = ["text_material_helpers.cc"],
    hdrs = ["text_material_helpers.h"],
    deps = ["//core/view/utils:view_config_proto_cc_imp"],
    alwayslink = 1,
)

proto_library(
    name = "text_collider_proto",
    srcs = [
        "text_collider_state.proto",
    ],
)

imp_cc_proto_library(
    name = "text_collider_imp_cc_proto",
    proto = ":text_collider_proto",
)

# --android_ndk_min_sdk_version=23

# --config=ios_x86_64

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

# --android_ndk_min_sdk_version=23

# --config=ios_x86_64

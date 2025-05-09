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
load(
    "//build_tools:imp.bzl",
    "if_android",
    "if_dev_runtime",
    "imp_defines",
    "imp_embed_generic_materials",
    "imp_google3_copts",
)
load(
    "//build_tools:imp_gstatic_urls.bzl",
    "imp_gstatic_urls",
)
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

GENERIC_MATERIALS_ZIP = "//core/loader/data:compiled_imp_default_gltf_materials.zip"

GENERIC_LITE_MATERIALS_ZIP = "//core/loader/data:compiled_imp_default_lite_gltf_materials.zip"

imp_assets(
    name = "generic_materials",
    srcs = [GENERIC_MATERIALS_ZIP],
    embed = imp_embed_generic_materials(),
    namespace = "imp::materials",
    urls = imp_gstatic_urls(
        [GENERIC_MATERIALS_ZIP],
        ["compiled_imp_default_gltf_materials"],
    ),
)

imp_assets(
    name = "generic_lite_materials",
    srcs = [GENERIC_LITE_MATERIALS_ZIP],
    embed = imp_embed_generic_materials(),
    namespace = "imp::materials",
    urls = imp_gstatic_urls(
        [GENERIC_LITE_MATERIALS_ZIP],
        ["compiled_imp_default_lite_gltf_materials"],
    ),
)

# Version of generic materials assets that are always embedded regardless of build flags.
# This is used in tests to embed required assets that would normally be downloaded.
# See mock_url_loader.h for how this is used.
imp_assets(
    name = "generic_materials_always_embedded",
    srcs = [GENERIC_MATERIALS_ZIP],
    base_url = "https://www.gstatic.com/ar/core/viewer",
    embed = True,
    namespace = "imp::materials_embedded",
)

imp_assets(
    name = "generic_lite_materials_always_embedded",
    srcs = [GENERIC_LITE_MATERIALS_ZIP],
    base_url = "https://www.gstatic.com/ar/core/viewer",
    embed = True,
    namespace = "imp::materials_embedded",
)

cc_library(
    name = "framework",
    srcs = glob(["**/*.cc"]),
    hdrs = glob(["**/*.h"]),
    copts = imp_google3_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":framework_proto_cc",
        ":generic_lite_materials",
        ":generic_materials",
        "//core:config",
        "//core/animation:fbs",
        "//core/animation:gltf_animation",
        "//core/animation:light_animation",
        "//core/animation:material_animation",
        "//core/animation:morph_target_animation",
        "//core/animation:texture_transform_animation",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/assets/material:material_helpers",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:executor",
        "//core/async:future",
        "//core/async:future_status_utils",
        "//core/async:thread_pool_executor",
        "//core/collision:bvh",
        "//core/collision:collider_helpers",
        "//core/collision:collision_accelerator_provider",
        "//core/collision:collision_flags",
        "//core/collision:collision_helpers",
        "//core/collision:mesh_collision_accelerator",
        "//core/collision:plane",
        "//core/collision:ray",
        "//core/common",
        "//core/common:bit_flag",
        "//core/common:bit_vector",
        "//core/common:buffer_access",
        "//core/common:context",
        "//core/common:debug_draw",
        "//core/common:fbs",
        "//core/common:filament_engine_helpers",
        "//core/common:filament_helpers",
        "//core/common:hash",
        "//core/common:invocable",
        "//core/common:log",
        "//core/common:optional_error",
        "//core/common:owned_or_borrowed_ptr",
        "//core/common:owned_or_unowned_memory",
        "//core/common:owned_ptr",
        "//core/common:pass_key",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/common:status_helpers",
        "//core/common:trace",
        "//core/common:tuple_helpers",
        "//core/common:type_traits",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/common:zip_helpers",
        "//core/geometry",
        "//core/geometry/shapes",
        "//core/graph:dependency_graph",
        "//core/image",
        "//core/image:decode_image",
        "//core/input",
        "//core/lighting:environment_light",
        "//core/lighting:environment_light_factory",
        "//core/lighting:image_based_lighting_asset",
        "//core/loader",
        "//core/loader:loader_creator",
        "//core/loader:loader_details",
        "//core/loader:loader_in_process",
        "//core/loader:loader_options",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/loader/provider:loaded_model_builder",
        "//core/material_library:generic_material",
        "//core/material_library:generic_material_constants",
        "//core/material_library:generic_material_parameters",
        "//core/material_library:material_package",
        "//core/material_library:material_param_value",
        "//core/materials",
        "//core/math",
        "//core/math:aabb_helpers",
        "//core/media",
        "//core/model",
        "//core/model:model_data_details",
        "//core/model/mesh",
        "//core/model/mesh:base_mesh_builder",
        "//core/model/mesh:mesh_builder",
        "//core/model/mesh:mesh_data",
        "//core/monitor",
        "//core/monitor:frame_loop_watcher",
        "//core/ncsb",
        "//core/ncsb:ncsb_proto_cc",
        "//core/ncsb:node_flag",
        "//core/proto:any_imp_proto",
        "//core/proto:proto_reader",
        "//core/proto:proto_writer",
        "//core/proto:textproto_reader",
        "//core/proto:textproto_writer",
        "//core/render",
        "//core/render:base_renderable_manager",
        "//core/render:shader_cache_system",
        "//core/resources",
        "//core/resources:url_loader",
        "//core/scene_handles:scene_handle_interface",
        "//core/split_engine:split_engine_serializer",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/async:future",
        "//core/view/platforms/xr_android:xr_color_space_helper",
        "//core/view/scripting",
        "//core/view/utils:asset",
        "//core/view/utils:default_view_config",
        "//core/view/utils:device",
        "//core/view/utils:frame_time",
        "//core/view/utils:macros",
        "//core/view/utils:render_setting_utils",
        "//core/view/utils:string_map",
        "//core/view/utils:view_config_proto_cc_imp",
        "//core/window:default_lighting",
        "//core/window:filament_host",
        "//core/window:loose_headers",
        "//core/window:window_rotation",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:any",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/types:variant",
        "@mediapipe//mediapipe/framework/deps:clock",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ] + if_dev_runtime([
        "@dear_imgui",
        "//core/editor:command",
        "//core/editor:editor_field_control",
        "//core/editor:node_value_command",
        "//core/editor/ui:filterable_combo",
        "//core/editor/widgets:component_widgets",
        "//core/proto:json_reader",
        "//core/proto:json_writer",
    ]) + if_android([
        "//core/async:future_interrupter",
        "//core/view/platforms/android/wrappers:input_stream",
    ]),
    alwayslink = 1,
)

proto_library(
    name = "framework_proto",
    srcs = glob(["**/*.proto"]),
    deps = [
        "//core/math:arrays_proto",
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "framework_proto_cc",
    proto = ":framework_proto",
    deps = [
        "//core/math:arrays_proto_cc",
        "//core/math:math_proto_cc",
        "//core/ncsb",
        "//core/proto:any_imp_proto",
        "//core/resources",
    ],
)

java_lite_proto_library(
    name = "framework_java_proto_lite",
    deps = [":framework_proto"],
)

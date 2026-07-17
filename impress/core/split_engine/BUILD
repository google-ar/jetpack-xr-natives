load(
    "@flatbuffers//:build_defs.bzl",
    "flatbuffer_cc_library",
)

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
load(
    "//build_tools:imp.bzl",
    "IMP_FLATBUFFER_INCLUDE_PATHS",
    "IMP_FLATC_ARGS",
    "if_android",
    "if_dev_runtime",
    "if_non_android",
    "imp_copts",
    "imp_defines",
    "imp_test_copts",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

alias(
    name = "fbs",
    actual = "@third_party//split_engine:fbs",
)

alias(
    name = "fbs_deps",
    actual = "@third_party//split_engine:fbs_deps",
)

config_setting(
    name = "imp_split_engine_allow_experimental_apis",
    define_values = {
        "IMP_SPLIT_ENGINE_ALLOW_EXPERIMENTAL_APIS": "1",
    },
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**"]),
)

cc_library(
    name = "message_group_monitor",
    srcs = ["message_group_monitor.cc"],
    hdrs = ["message_group_monitor.h"],
    copts = imp_copts(),
    deps = [
        ":split_engine_shared",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:no_destructor",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_bridge_sender",
    hdrs = ["split_engine_bridge_sender.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_shared",
        "//core/async:background_scheduler",
        "//core/common:owned_ptr",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_bridge_sender_mock",
    testonly = True,
    hdrs = [
        "split_engine_bridge_sender_mock.h",
    ],
    copts = imp_test_copts(),
    defines = imp_defines(),
    deps = [
        ":flatbuffer_allocator",
        ":split_engine_bridge_sender",
        ":split_engine_renderer",
        ":split_engine_shared",
        "//core/common",
        "//core/common:invocable",
        "//core/common:owned_ptr",
        "//core/common:registry",
        "//core/resources",
        "//core/view:base_view",
        "//core/view/framework",
        "//testing:base_executor_test_helper",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@third_party//split_engine:fbs",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "split_engine_shared",
    hdrs = [
        "shared/split_engine_defines.h",
    ],
    copts = imp_copts(),
    deps = [
        "//core/common:enum_flags",
    ],
    alwayslink = 1,
)

cc_library(
    name = "renderer_policy_handler",
    hdrs = [
        "renderer_policy_handler.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/lighting:environment_light",
        "//core/ncsb",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "renderer_policy_handler_mock",
    testonly = True,
    hdrs = [
        "renderer_policy_handler_mock.h",
    ],
    copts = imp_test_copts(),
    defines = imp_defines(),
    deps = [
        ":renderer_policy_handler",
        "//core/lighting:environment_light",
        "//core/ncsb",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@mediapipe//mediapipe/framework/port:gtest",
    ],
    alwayslink = 1,
)

cc_library(
    name = "user_id_holder",
    srcs = ["user_id_holder.cc"],
    hdrs = ["user_id_holder.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/ncsb",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_renderer",
    hdrs = ["split_engine_renderer.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":renderer_policy_handler",
        ":split_engine_renderer_context",
        ":split_engine_shared",
        "//core/async:future",
        "//core/common:invocable",
        "//core/media:media_color_space",
        "//core/ncsb",
        "//core/render",
        "//core/render:content_security_level",
        "//core/render:display_color_space",
        "//core/render/android:platform_android_external_texture_surface",
        "//core/split_engine/materials/builtin:builtin_material",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_renderer_mock",
    testonly = True,
    hdrs = [
        "split_engine_renderer_mock.h",
    ],
    copts = imp_test_copts(),
    defines = imp_defines(),
    deps = [
        ":renderer_policy_handler",
        ":split_engine_renderer",
        ":split_engine_renderer_context",
        ":split_engine_shared",
        "//core/async:future",
        "//core/common:invocable",
        "//core/media:media_color_space",
        "//core/ncsb",
        "//core/render:content_security_level",
        "//core/render:display_color_space",
        "//core/render/android:platform_android_external_texture_surface",
        "//core/split_engine/materials/builtin:builtin_material",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "args_validator",
    srcs = [
        "renderable_validator.cc",
        "texture_validator.cc",
    ],
    hdrs = [
        "renderable_validator.h",
        "texture_validator.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":fbs",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_node_info",
    hdrs = ["split_engine_node_info.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core:config",
        "//core/ncsb",
    ] + if_dev_runtime([
        "@dear_imgui",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "split_engine_serializer",
    srcs = ["split_engine_serializer.cc"],
    hdrs = ["split_engine_serializer.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_mesh_serializer",
        ":split_engine_texture_serializer",
        "//core:config",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:invocable",
        "//core/geometry/shapes",
        "//core/lighting:image_based_lighting_types",
        "//core/material_library:fbs",
        "//core/material_library:fbs_deps",
        "//core/material_library:generic_material_spec",
        "//core/material_library:material_param_value",
        "//core/math",
        "//core/model/mesh:base_mesh_builder",
        "//core/render:base_texture_builder",
        "//core/render_passes:texture_pipeline_renderer_projection_quad",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine:split_engine_schema_version",
    ] + if_android([
        "//core/render/android:definitions",
        "//core/render/android:platform_android_external_texture_surface",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "split_engine_serializer_transport",
    hdrs = ["split_engine_serializer_transport.h"],
    copts = imp_copts(),
    deps = [
        ":split_engine_shared",
        "//core/async:background_scheduler",
        "//core/common:invocable",
        "//core/common:owned_ptr",
        "//core/common:pass_key",
        "//core/split_engine/android:split_engine_android_bridge",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_serializer_transport_legacy_impl",
    srcs = ["split_engine_serializer_transport_legacy_impl.cc"],
    hdrs = ["split_engine_serializer_transport_legacy_impl.h"],
    copts = imp_copts(),
    deps = [
        ":split_engine_bridge_sender",
        ":split_engine_serializer_transport",
        ":split_engine_shared",
        "//core/common:invocable",
        "//core/common:owned_ptr",
        "//core/common:pass_key",
        "//core/split_engine/android:split_engine_android_bridge",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_texture_serializer",
    hdrs = ["split_engine_texture_serializer.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "@flatbuffers",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_mesh_serializer",
    hdrs = ["split_engine_mesh_serializer.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":flatbuffer_size_calculator",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "texture_pipeline_schema_conversion",
    srcs = ["texture_pipeline_schema_conversion.cc"],
    hdrs = ["texture_pipeline_schema_conversion.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":flatbuffer_utils",
        ":split_engine_renderer_context",
        ":split_engine_shared",
        "//core/common:type_helpers",
        "//core/math",
        "//core/ncsb",
        "//core/render_passes:texture_pipeline_renderer_proto_cc_imp",
        "//core/view/utils:render_settings_proto_cc_imp",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_serializer_impl",
    srcs = [
        "split_engine_renderable_builder.cc",
        "split_engine_serializer_batch_manager.cc",
        "split_engine_serializer_data_types.cc",
        "split_engine_serializer_impl.cc",
    ],
    hdrs = [
        "split_engine_renderable_builder.h",
        "split_engine_serializer_batch_manager.h",
        "split_engine_serializer_data_types.h",
        "split_engine_serializer_impl.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":flatbuffer_allocator",
        ":flatbuffer_size_calculator",
        ":flatbuffer_utils",
        ":image_based_lighting_helpers",
        ":split_engine_bridge_sender",
        ":split_engine_mesh_builder",
        ":split_engine_mesh_serializer",
        ":split_engine_serializer",
        ":split_engine_serializer_transport",
        ":split_engine_shared",
        ":split_engine_texture_builder",
        ":split_engine_texture_serializer",
        ":texture_pipeline_schema_conversion",
        "//core:config",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:entity_absl_hasher",
        "//core/common:enum_flags",
        "//core/common:fbs",
        "//core/common:invocable",
        "//core/common:owned_ptr",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/common:type_helpers",
        "//core/geometry/shapes",
        "//core/lighting:image_based_lighting_types",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/loader/provider:loaded_model_builder",
        "//core/material_library:fbs",
        "//core/material_library:fbs_deps",  # build_cleaner: keep
        "//core/material_library:flatbuffer_utils",
        "//core/material_library:generic_material_spec",
        "//core/material_library:material_param_value",
        "//core/materials",
        "//core/math",
        "//core/math:flatbuffer_support",
        "//core/model/mesh:base_mesh_builder",
        "//core/ncsb",
        "//core/render",
        "//core/render:base_renderable_manager",
        "//core/render:base_texture_builder",
        "//core/render_passes:texture_pipeline_renderer",
        "//core/render_passes:texture_pipeline_renderer_projection_quad",
        "//core/render_passes:texture_pipeline_renderer_proto_cc_imp",
        "//core/split_engine/android:split_engine_android_bridge",
        "//core/split_engine/materials:builtin_texture_parameter_creator",
        "//core/split_engine/materials:split_engine_builtin_material",
        "//core/split_engine/materials:split_engine_custom_material",
        "//core/split_engine/materials:split_engine_generic_material",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "//core/view/utils:render_settings_proto_cc_imp",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ] + if_android([
        "//core/render/android:definitions",
        "//core/render/android:platform_android_external_texture_surface",
        "//core/split_engine/android:split_engine_platform_android_external_texture_surface",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "split_engine_mesh_builder",
    srcs = [
        "split_engine_mesh_builder.cc",
    ],
    hdrs = [
        "split_engine_mesh_builder.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":flatbuffer_size_calculator",
        ":split_engine_mesh_serializer",
        ":split_engine_serializer",
        "//core/loader:loader_options",
        "//core/math",
        "//core/model/mesh:base_mesh_builder",
        "//core/model/mesh:mesh_builder",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_mesh_factory",
    srcs = [
        "split_engine_mesh_factory.cc",
    ],
    hdrs = [
        "split_engine_mesh_factory.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":fbs",
        ":fbs_deps",  # buildcleaner: keep
        "//core/common:fbs",
        "//core/common:type_helpers",
        "//core/common:typed_ids",
        "//core/loader:loader_details",
        "//core/loader:loader_options",
        "//core/loader/creator:inflight_creation",
        "//core/loader/creator:model_creator_helper",
        "//core/math",
        "//core/model/mesh:base_mesh_builder",
        "//core/model/mesh:mesh_builder",
        "//core/model/mesh:mesh_data",
        "//core/view:base_view",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:filabridge_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_texture_builder",
    srcs = [
        "split_engine_texture_builder.cc",
    ],
    hdrs = [
        "split_engine_texture_builder.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":flatbuffer_size_calculator",
        ":split_engine_serializer",
        ":split_engine_texture_serializer",
        "//core/assets",
        "//core/async:executor",
        "//core/image",
        "//core/render",
        "//core/render:base_texture_builder",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
        "@third_party//split_engine:split_engine_schema_version",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_serializer_mock",
    testonly = True,
    hdrs = ["split_engine_serializer_mock.h"],
    copts = imp_test_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_mesh_serializer",
        ":split_engine_serializer",
        ":split_engine_texture_serializer",
        "//core:config",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:invocable",
        "//core/common:small_source_location",
        "//core/common:typed_ids",
        "//core/geometry/shapes",
        "//core/lighting:image_based_lighting_types",
        "//core/material_library:fbs_deps",  # buildcleaner: keep
        "//core/material_library:generic_material",
        "//core/material_library:generic_material_parameters",
        "//core/material_library:generic_material_spec",
        "//core/material_library:material_param_value",
        "//core/materials",
        "//core/math",
        "//core/model:model_data_details",
        "//core/model/mesh:base_mesh_builder",
        "//core/render",
        "//core/render:base_texture_builder",
        "//core/split_engine/android:split_engine_android_bridge",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/functional:function_ref",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
        "@third_party//split_engine:split_engine_schema_version",
    ] + if_android([
        "//core/render/android:android_external_texture_surface",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "flatbuffer_allocator",
    srcs = ["flatbuffer_arena_allocator.cc"],
    hdrs = ["flatbuffer_arena_allocator.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@com_google_zetasql//zetasql/base:arena",
        "@flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_renderable_info",
    srcs = ["split_engine_renderable_info.cc"],
    hdrs = ["split_engine_renderable_info.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":skinning_helpers",
        ":split_engine_filament_resource_ptrs",
        ":split_engine_shared",
        "//core:config",
        "//core/common:small_source_location",
        "//core/materials",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/render:base_renderable_manager",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@third_party//filament/filament:filament_hdrs",
    ] + if_dev_runtime([
        "@dear_imgui",
        "//core/render",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "split_engine_mesh_collider",
    srcs = ["split_engine_mesh_collider.cc"],
    hdrs = ["split_engine_mesh_collider.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":split_engine_renderable_info",
        "//core/collision:collision_flags",
        "//core/collision:collision_helpers",
        "//core/collision:ray",
        "//core/common:debug_draw",
        "//core/geometry/shapes",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/render:base_renderable_manager",
        "//core/view/framework",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "flatbuffer_size_calculator",
    srcs = ["flatbuffer_size_calculator.cc"],
    hdrs = ["flatbuffer_size_calculator.h"],
    copts = imp_copts(),
    linkstatic = 1,
    alwayslink = 1,
)

cc_library(
    name = "flatbuffer_utils",
    srcs = ["flatbuffer_utils.cc"],
    hdrs = ["flatbuffer_utils.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        "//core/assets/material:material_load_options_proto_cc",
        "//core/math",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "skinning_helpers",
    srcs = ["skinning_helpers.cc"],
    hdrs = ["skinning_helpers.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        "//core/math",
        "//core/model/mesh:mesh_data",
        "@com_google_absl//absl/log:check",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_test_fixture",
    testonly = True,
    hdrs = ["split_engine_test_fixture.h"],
    deps = [
        ":renderer_policy_handler_mock",
        ":split_engine_renderer",
        ":split_engine_renderer_impl",
        ":split_engine_serializer",
        ":split_engine_serializer_impl",
        ":split_engine_serializer_transport_legacy_impl",
        ":split_engine_shared",
        ":split_engine_test_bridge",
        "//:testing_api",
        "//core/async:executor",
        "//core/common:enum_flags",
        "//core/common:invocable",
        "//core/common:owned_ptr",
        "//core/lighting:environment_light",
        "//core/material_library:generic_materials",
        "//core/material_library:generic_materials_always_embedded",
        "//core/split_engine/android:split_engine_shared_memory_bridge_client_mock",
        "//core/split_engine/test_assets",
        "//core/view:base_view",
        "//core/view/framework",
        "//testing",
        "//testing:base_scuba_view_fixture",
        "//testing:scuba_view_fixture",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

flatbuffer_cc_library(
    name = "split_engine_test_fbs",
    srcs = ["schema_test_data/bridge_sequence.fbs"],
    flatc_args = IMP_FLATC_ARGS,
    include_paths = IMP_FLATBUFFER_INCLUDE_PATHS,
    includes = [
        "@third_party//split_engine:schemas",
    ],
    out_prefix = "schema_test_data/",
)

cc_library(
    name = "split_engine_test_fbs_deps",
    deps = [
        ":split_engine_test_fbs",
        "@third_party//split_engine:fbs_deps",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_test_bridge",
    testonly = True,
    srcs = [
        "split_engine_test_bridge.cc",
        "split_engine_test_bridge_serializer.cc",
    ],
    hdrs = [
        "split_engine_test_bridge.h",
        "split_engine_test_bridge_serializer.h",
    ],
    copts = imp_copts(),
    data = glob(["schema_test_data/**"]),
    defines = imp_defines(),
    # need to link statically because utils::ashmem_create_region is not marked as exported
    linkstatic = 1,
    local_defines = [
        "SCHEMA_TEST_DATA_DIR=\\\"" + package_name() + "/schema_test_data\\\"",
    ],
    deps = [
        ":flatbuffer_allocator",
        ":flatbuffer_size_calculator",
        ":split_engine_bridge_sender",
        ":split_engine_renderer",
        ":split_engine_serializer",
        ":split_engine_shared",
        ":split_engine_test_fbs",
        ":split_engine_test_fbs_deps",  # buildcleaner: keep
        "//core/async:executor",
        "//core/common",
        "//core/common:invocable",
        "//core/common:owned_ptr",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/render:base_renderable_manager",
        "//core/resources",
        "//core/split_engine/android:split_engine_android_bridge",
        "//core/split_engine/android:split_engine_shared_memory_bridge_client",
        "//core/split_engine/android:split_engine_shared_memory_bridge_client_mock",
        "//core/split_engine/android:split_engine_shared_memory_bridge_service_impl",
        "//core/view:base_view",
        "//core/view/framework",
        "//testing:base_executor_test_helper",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/functional:any_invocable",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:split_engine_schema_version",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

filegroup(
    name = "split_engine_fuzz_corpus",
    srcs = glob([
        "fuzz_corpus/**",
    ]),
)

cc_library(
    name = "image_based_lighting_helpers",
    srcs = ["image_based_lighting_helpers.cc"],
    hdrs = ["image_based_lighting_helpers.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        "//core/image",
        "//core/lighting:image_based_lighting_types",
        "//core/math",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "flatbuffers_fuzzing_domains",
    srcs = ["flatbuffers_fuzzing_domains.cc"],
    hdrs = ["flatbuffers_fuzzing_domains.h"],
    deps = [
        "//testing/fuzzing:domains",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/random:bit_gen_ref",
        "@com_google_absl//absl/random:distributions",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_filament_resource_ptrs",
    hdrs = ["split_engine_filament_resource_ptrs.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        "//core/common:owned_ptr",
        "//core/view:base_view",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "flatbuffers_attributes_validator",
    srcs = ["flatbuffers_attributes_validator.cc"],
    hdrs = ["flatbuffers_attributes_validator.h"],
    copts = imp_copts(),
    deps = [
        "//core/common:invocable",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_renderer_context",
    srcs = ["split_engine_renderer_context.cc"],
    hdrs = ["split_engine_renderer_context.h"],
    copts = imp_copts(),
    deps = [
        ":renderer_policy_handler",
        ":split_engine_filament_resource_ptrs",
        ":split_engine_renderable_info",
        ":split_engine_shared",
        ":user_id_holder",
        "//core/assets",
        "//core/common:invocable",
        "//core/common:rememberer",
        "//core/common:robin_map",
        "//core/lighting:environment_light",
        "//core/lighting:image_based_lighting_asset",
        "//core/materials",
        "//core/math",
        "//core/media:media_color_space",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine/materials/builtin:builtin_material",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_shared_context",
    srcs = [],
    hdrs = ["split_engine_shared_context.h"],
    copts = imp_copts(),
    deps = [
        "//core/split_engine/materials:split_engine_builtin_material_factory",
        "//core/view:base_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_external_texture_color_space_store",
    srcs = ["split_engine_external_texture_color_space_store.cc"],
    hdrs = ["split_engine_external_texture_color_space_store.h"],
    copts = imp_copts(),
    deps = [
        ":split_engine_shared",
        "//core/media:media_color_space",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

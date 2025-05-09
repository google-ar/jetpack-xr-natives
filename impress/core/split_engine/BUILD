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
load(
    "@com_github_google_flatbuffers//:build_defs.bzl",
    "flatbuffer_cc_library",
)
load(
    "//build_tools:imp.bzl",
    "IMP_FLATBUFFER_INCLUDE_PATHS",
    "IMP_FLATC_ARGS",
    "if_android",
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

cc_library(
    name = "split_engine_bridge_sender",
    hdrs = [
        "split_engine_bridge_sender.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":flatbuffer_allocator",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_bridge_sender_mock",
    testonly = True,
    hdrs = [
        "split_engine_bridge_sender_mock.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":flatbuffer_allocator",
        ":split_engine_bridge_sender",
        ":split_engine_renderer",
        ":split_engine_shared",
        "//core/common",
        "//core/common:log",
        "//core/common:registry",
        "//core/resources",
        "//core/view:base_view",
        "//core/view/framework",
        "//testing:base_executor_test_helper",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
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
    copts = imp_copts(),
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
        ":split_engine_shared",
        "//core/async:future",
        "//core/common:invocable",
        "//core/render",
        "//core/render:content_security_level",
        "//core/render:display_color_space",
        "//core/render/android:platform_android_external_texture_surface",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
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
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_renderer_impl",
    srcs = ["split_engine_renderer_impl.cc"],
    hdrs = ["split_engine_renderer_impl.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":args_validator",
        ":fbs",
        ":flatbuffer_utils",
        ":image_based_lighting_helpers",
        ":renderer_policy_handler",
        ":split_engine_filament_resource_ptrs",
        ":split_engine_mesh_collider",
        ":split_engine_mesh_factory",
        ":split_engine_node_info",
        ":split_engine_renderable_info",
        ":split_engine_renderer",
        ":split_engine_shared",
        ":user_id_holder",
        "//core/assets",
        "//core/assets/material:material_asset",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:invocable",
        "//core/common:log",
        "//core/common:owned_ptr",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/common:robin_map",
        "//core/common:trace",
        "//core/geometry/shapes",
        "//core/lighting:environment_light",
        "//core/lighting:environment_light_factory",
        "//core/lighting:image_based_lighting_asset",
        "//core/lighting:image_based_lighting_types",
        "//core/loader:loader_options",
        "//core/loader/creator:inflight_creation",
        "//core/material_library:flatbuffer_utils",
        "//core/materials",
        "//core/math",
        "//core/model:model_data_details",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/render",
        "//core/render:base_renderable_manager",
        "//core/render:content_security_level",
        "//core/render:display_color_space",
        "//core/render/android:platform_android_external_texture_surface",
        "//core/split_engine/materials:split_engine_material_factory",
        "//core/split_engine/materials/builtin:builtin_material",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_node_info",
    hdrs = ["split_engine_node_info.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/ncsb",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_serializer",
    hdrs = ["split_engine_serializer.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_mesh_serializer",
        ":split_engine_texture_serializer",
        "//core:config",
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
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ] + if_android([
        "//core/render/android:definitions",
        "//core/render/android:platform_android_external_texture_surface",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "split_engine_texture_serializer",
    hdrs = ["split_engine_texture_serializer.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "@com_github_google_flatbuffers//:flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
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
        "@com_github_google_flatbuffers//:flatbuffers",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_serializer_impl",
    srcs = ["split_engine_serializer_impl.cc"],
    hdrs = ["split_engine_serializer_impl.h"],
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
        ":split_engine_shared",
        ":split_engine_texture_builder",
        ":split_engine_texture_serializer",
        "//core:config",
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:entity_absl_hasher",
        "//core/common:enum_flags",
        "//core/common:fbs",
        "//core/common:log",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:robin_map",
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
        "//core/math",
        "//core/math:flatbuffer_support",
        "//core/model/mesh:base_mesh_builder",
        "//core/ncsb",
        "//core/render:base_renderable_manager",
        "//core/render:base_texture_builder",
        "//core/split_engine/materials:split_engine_generic_material",
        "//core/split_engine/materials:split_engine_material",
        "//core/split_engine/materials:split_engine_material_factory",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/time",
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
        "//core/common:log",
        "//core/loader:loader_options",
        "//core/math",
        "//core/model/mesh:base_mesh_builder",
        "//core/model/mesh:mesh_builder",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings:string_view",
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
        "//core/common:log",
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
        ":split_engine_serializer",
        ":split_engine_shared",
        ":split_engine_texture_serializer",
        "//core/image",
        "//core/render:base_texture_builder",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/strings:string_view",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
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
        "//core/async:future",
        "//core/common:buffer_access",
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
        "//core/model/mesh:base_mesh_builder",
        "//core/render",
        "//core/render:base_texture_builder",
        "//core/view/utils:string_map",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
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
        "//core/common:log",
        "//core/common:platform_helpers",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_zetasql//zetasql/base:arena",
        "@third_party//filament/libs:utils_hdrs",
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
        "//core/materials",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/render:base_renderable_manager",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@third_party//filament/filament:filament_hdrs",
    ],
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
        "//core/common:log",
        "//core/geometry/shapes",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/render:base_renderable_manager",
        "//core/view/framework",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
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
        ":split_engine_serializer",
        "//core/material_library:flatbuffer_utils",
        "//core/math",
        "//core/render",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@third_party//filament/filament:filament_hdrs",
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
        ":split_engine_shared",
        ":split_engine_test_bridge",
        "//:testing_api",
        "//core/async:executor",
        "//core/common:enum_flags",
        "//core/common:invocable",
        "//core/common:log",
        "//core/lighting:environment_light",
        "//core/split_engine/android:split_engine_android_bridge",
        "//core/split_engine/test_assets",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:generic_materials",
        "//core/view/framework:generic_materials_always_embedded",
        "//testing",
        "//testing:scuba_view_fixture",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
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
    local_defines = [
        "SCHEMA_TEST_DATA_DIR=\\\"" + package_name() + "/schema_test_data\\\"",
    ],
    deps = [
        ":flatbuffer_allocator",
        ":split_engine_bridge_sender",
        ":split_engine_renderer",
        ":split_engine_shared",
        ":split_engine_test_fbs",
        ":split_engine_test_fbs_deps",  # buildcleaner: keep
        "//core/async:executor",
        "//core/common",
        "//core/common:invocable",
        "//core/common:log",
        "//core/common:registry",
        "//core/render:base_renderable_manager",
        "//core/resources",
        "//core/split_engine/android:split_engine_android_bridge",
        "//core/split_engine/android:split_engine_shared_memory_bridge_client",
        "//core/split_engine/android:split_engine_shared_memory_bridge_client_mock",
        "//core/split_engine/android:split_engine_shared_memory_bridge_service_impl",
        "//core/view:base_view",
        "//core/view/framework",
        "//testing:base_executor_test_helper",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@mediapipe//mediapipe/framework/port:status",
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
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
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

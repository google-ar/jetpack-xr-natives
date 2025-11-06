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
    "@flatbuffers//:build_defs.bzl",
    "flatbuffer_cc_library",
)
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "IMP_FLATBUFFER_INCLUDE_PATHS", "IMP_FLATC_ARGS", "imp_copts", "imp_defines", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

loader_schemas = [
    "schemas/loaded_model.fbs",
]

filegroup(
    name = "loader_schemas",
    srcs = loader_schemas,
)

flatbuffer_cc_library(
    name = "fbs",
    srcs = loader_schemas,
    flatc_args = IMP_FLATC_ARGS,
    include_paths = IMP_FLATBUFFER_INCLUDE_PATHS,
    includes = [
        "//core/animation:schemas",
        "//core/common:schemas",
        "//core/loader/provider/extensions/interactivity:schemas",
        "//core/material_library:schemas",
    ],
    out_prefix = "schemas/",
)

cc_library(
    name = "fbs_deps",
    deps = [
        ":fbs",
        "//core/animation:fbs_deps",
        "//core/common:fbs_deps",
        "//core/loader/provider/extensions/interactivity:fbs_deps",
        "//core/material_library:fbs_deps",
    ],
    alwayslink = 1,
)

cc_library(
    name = "provider",
    srcs = [
        "provider.cc",
    ],
    hdrs = [
        "provider.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":details",
        ":fbs",
        "//core/common",
        "//core/common:flatbuffer_helpers",
        "//core/common:zip_helpers",
        "//core/loader:loader_options",
        "//core/loader/provider/gltf:provider_gltf",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "provider_gltf_test_helpers",
    testonly = 1,
    srcs = ["provider_gltf_test_helpers.cc"],
    hdrs = ["provider_gltf_test_helpers.h"],
    copts = imp_google3_copts() + [
        # Gmock include order produces fatal error
        # -Wmacro-redefined. For now, emit as warning.
        "-Wno-macro-redefined",
    ],
    linkstatic = True,
    deps = [
        ":fbs",
        ":provider",
        "//core/common",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/common:flatbuffer_helpers",
        "//core/common:test_helpers",
        "//core/loader:loader_options",
        "//core/loader/data:embedded_imp_default_gltf_materials",
        "//core/loader/data:embedded_placeholder_textures",
        "//devtools/build/runtime:get_runfiles_dir",
        "@com_google_absl//absl/strings:string_view",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "provider_usdz_test_helpers",
    testonly = 1,
    srcs = ["provider_usdz_test_helpers.cc"],
    hdrs = ["provider_usdz_test_helpers.h"],
    copts = imp_google3_copts() + [
        # Gmock include order produces fatal error
        # -Wmacro-redefined. For now, emit as warning.
        "-Wno-macro-redefined",
    ],
    linkstatic = True,
    deps = [
        ":fbs",
        ":provider",
        "//core/common",
        "//core/common:buffer_access",
        "//core/common:optional_error",
        "//core/common:resource_helpers",
        "//core/common:test_helpers",
        "//core/loader:loader_options",
        "//core/loader/data:embedded_imp_default_gltf_materials",
        "//core/loader/data:embedded_placeholder_textures",
        "//core/loader/provider/usdz",
        "//devtools/build/runtime:get_runfiles_dir",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "details",
    srcs = [
        "details/loaded_zip.cc",
        "details/provider_details_common.cc",
    ],
    hdrs = [
        "details/gltf_provider.h",
        "details/loaded_zip.h",
        "details/provider_constants.h",
        "details/provider_details_common.h",
        "details/usdz_provider.h",
        "details/vertex_attribute.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":fbs",
        "//core/common",
        "//core/common:flatbuffer_helpers",
        "//core/common:string_helpers",
        "//core/common:zip_helpers",
        "//core/loader:loader_details",  # build_cleaner: keep
        "//core/loader:loader_options",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:variant",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "loaded_model_builder",
    srcs = [
        "details/loaded_model_builder.cc",
    ],
    hdrs = [
        "details/loaded_model_builder.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":details",
        ":fbs",
        ":fbs_deps",  # build_cleaner: keep
        "//core/animation:fbs",
        "//core/common:bit_vector",
        "//core/common:buffer_access",
        "//core/common:data_helpers",
        "//core/common:fbs",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/common:filament_helpers",
        "//core/common:flatbuffer_helpers",
        "//core/common:robin_map",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/image",
        "//core/loader/provider/extensions:extension_behavior",
        "//core/loader/provider/extensions:extension_interactivity",
        "//core/loader/provider/extensions/behavior:loader_extension",
        "//core/loader/provider/extensions/interactivity:fbs",
        "//core/loader/provider/extensions/interactivity:fbs_deps",  # build_cleaner: keep
        "//core/loader/provider/extensions/interactivity:loader_extension",
        "//core/loader/provider/gltf:dense_data_access",
        "//core/loader/provider/gltf:imp_cc_schema",
        "//core/material_library:fbs",
        "//core/material_library:generic_material_spec",
        "//core/math",
        "//core/math:flatbuffer_support",
        "//core/model",
        "//core/model:model_data_details",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

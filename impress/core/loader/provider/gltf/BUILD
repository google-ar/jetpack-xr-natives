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

load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "imp_copts", "imp_defines")
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

proto_library(
    name = "schema",
    srcs = ["gltf.proto"],
    deps = [
        "//core/loader/provider/extensions/interactivity:interactivity_proto",
        "//core/proto:imp_proto",
    ],
)

cc_proto_library(
    name = "schema_cc_proto",
    deps = [":schema"],
)

imp_cc_proto_library(
    name = "imp_cc_schema",
    proto = ":schema",
    deps = [
        "//core/loader/provider/extensions/interactivity:interactivity_imp_cc_proto",
    ],
)

cc_library(
    name = "gltf_extension_names",
    hdrs = [
        "gltf_extension_names.h",
    ],
    copts = imp_copts(),
    deps = [
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "texturable_parameter",
    hdrs = [
        "texturable_parameter.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    alwayslink = 1,
)

cc_library(
    name = "gltf_lookup",
    srcs = [
        "gltf_attribute.cc",
        "gltf_helpers.cc",
        "gltf_lookup.cc",
        "gltf_lookup_animation_pointer.cc",
    ],
    hdrs = [
        "gltf_attribute.h",
        "gltf_helpers.h",
        "gltf_lookup.h",
        "gltf_lookup_animation_pointer.h",
        "parsed_gltf.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":gltf_extension_names",
        ":imp_cc_schema",
        ":texturable_parameter",
        "//core/common",
        "//core/common:bit_vector",
        "//core/common:fbs",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/common:robin_map",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/loader:loader_options",
        "//core/loader/provider:details",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/math",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:inlined_vector",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "parse_gltf",
    srcs = [
        "parse_gltf.cc",
    ],
    hdrs = [
        "parse_gltf.h",
    ],
    copts = imp_copts(),
    deps = [
        ":imp_cc_schema",
        "//core/common",
        "//core/common:fbs",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/loader/provider/extensions:extension_interactivity",
        "//core/proto:json_reader",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "dense_data_access",
    hdrs = [
        "dense_data_access.h",
    ],
    copts = imp_copts(),
    deps = [
        "//core/common:buffer_access",
        "@com_google_absl//absl/log:check",
    ],
    alwayslink = 1,
)

cc_library(
    name = "accessor_reader",
    srcs = [
        "accessor_reader.cc",
    ],
    hdrs = [
        "accessor_reader.h",
    ],
    copts = imp_copts(),
    deps = [
        ":dense_data_access",
        ":gltf_lookup",
        ":imp_cc_schema",
        "//core/common:buffer_access",
        "//core/common:fbs",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/common:platform_helpers",
        "//core/common:typed_ids",
        "//core/loader:loader_details",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/math",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "gltf",
    srcs = [
        "gltf_animation.cc",
        "gltf_geometry.cc",
        "gltf_texture.cc",
    ],
    hdrs = [
        "gltf_animation.h",
        "gltf_geometry.h",
        "gltf_texture.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":accessor_reader",
        ":dense_data_access",
        ":gltf_lookup",
        ":imp_cc_schema",
        "//core/common",
        "//core/common:data_helpers",
        "//core/common:fbs",
        "//core/common:filament_helpers",
        "//core/common:robin_map",
        "//core/geometry/shapes",
        "//core/image",
        "//core/loader:loader_options",
        "//core/loader/provider:loaded_model_builder",
        "//core/loader/provider/extensions:extension_basis",
        "//core/loader/provider/extensions:extension_mesh_features",
        "//core/loader/provider/extensions/interactivity:interactivity_imp_cc_proto",
        "//core/material_library:fbs",
        "//core/material_library:generic_material_constants",
        "//core/material_library:material_param_value",
        "//core/math",
        "//core/model",
        "//core/model:model_data_details",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/container:inlined_vector",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:geometry",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "provider_gltf",
    srcs = [
        "provider_gltf.cc",
    ],
    hdrs = [
        "provider_gltf.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":accessor_reader",
        ":dense_data_access",
        ":gltf",
        ":gltf_lookup",
        ":imp_cc_schema",
        ":parse_gltf",
        "//core/animation:fbs",
        "//core/animation:gltf_conversions",
        "//core/common",
        "//core/common:bit_vector",
        "//core/common:fbs",
        "//core/common:filament_helpers",
        "//core/common:flatbuffer_helpers",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/loader/provider:details",
        "//core/loader/provider:fbs",
        "//core/loader/provider:loaded_model_builder",
        "//core/loader/provider/extensions:extension_draco",
        "//core/loader/provider/extensions:extension_meshopt",
        "//core/loader/provider/extensions/interactivity:interactivity_imp_cc_proto",
        "//core/loader/provider/extensions/interactivity:loader_extension",
        "//core/material_library:generic_material_schema_provider",
        "//core/math",
        "//core/model",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

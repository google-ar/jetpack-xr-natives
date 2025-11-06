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
load("//build_tools:imp.bzl", "imp_copts", "imp_linkopts")
load("//build_tools:imp_extensions.bzl", "imp_extensions")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_extensions(
    name = "all_impl",
    extensions = [
        "basis",
        "draco",
        "meshopt",
        "verification",
        "behavior",
        "interactivity",
        "mesh_features",
    ],
)

imp_extensions(
    name = "all_noop",
    extensions = [],
)

cc_library(
    name = "extension_basis",
    hdrs = [
        "gltf_extension_basis.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        "//core/image",
        "//core/loader:loader_options",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_basis_impl",
    srcs = [
        "gltf_extension_basis.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_basis",
        "//core/image:ktx2_decode_image",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_basis_noop",
    srcs = [
        "gltf_extension_basis_noop.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_basis",
        "//core/common:optional_error",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_draco",
    hdrs = [
        "gltf_extension_draco.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        "//core/common:buffer_access",
        "//core/loader/provider/gltf:imp_cc_schema",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_draco_impl",
    srcs = [
        "gltf_extension_draco.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_draco",
        "//core/common:buffer_access",
        "//core/common:optional_error",
        "//core/loader/provider/gltf:imp_cc_schema",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@draco",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_draco_noop",
    srcs = [
        "gltf_extension_draco_noop.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_draco",
        "//core/common:buffer_access",
        "//core/common:optional_error",
        "//core/loader/provider/gltf:imp_cc_schema",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_meshopt",
    hdrs = [
        "gltf_extension_meshopt.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        "//core/common:buffer_access",
        "//core/loader/provider/gltf:imp_cc_schema",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_meshopt_impl",
    srcs = [
        "gltf_extension_meshopt.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_meshopt",
        "//core/common:buffer_access",
        "//core/loader/provider/gltf:imp_cc_schema",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@meshoptimizer",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_meshopt_noop",
    srcs = [
        "gltf_extension_meshopt_noop.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_meshopt",
        "//core/common:buffer_access",
        "//core/loader/provider/gltf:imp_cc_schema",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_mesh_features",
    hdrs = [
        "gltf_extension_mesh_features.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = ["//core/loader/provider/gltf:imp_cc_schema"],
    alwayslink = 1,
)

cc_library(
    name = "extension_mesh_features_impl",
    srcs = [
        "gltf_extension_mesh_features.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_mesh_features",
        "//core/loader/provider/gltf:imp_cc_schema",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_mesh_features_noop",
    srcs = [
        "gltf_extension_mesh_features_noop.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_mesh_features",
        "//core/loader/provider/gltf:imp_cc_schema",
    ],
    alwayslink = 1,
)

cc_library(
    name = "verification",
    hdrs = [
        "verification.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        "//core/common",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/common:flatbuffer_helpers",
        "//core/common:typed_set_vector",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/material_library:fbs",
        "//core/model",
        "//core/model:model_data_details",
        "@com_google_absl//absl/status",
        "@flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "verification_impl",
    srcs = [
        "verification.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":verification",
        "//core/common:buffer_access",
        "//core/common:enum_flags",
        "//core/common:fbs",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/common:flatbuffer_helpers",
        "//core/common:optional_error",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/loader:loaded_model_fb",
        "//core/loader:loader_details",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/material_library:fbs",
        "//core/model",
        "//core/model:model_data_details",
        "@com_google_absl//absl/status",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "verification_noop",
    srcs = [
        "verification_noop.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":verification",
        "//core/common:buffer_access",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/common:flatbuffer_helpers",
        "//core/common:typed_set_vector",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/material_library:fbs",
        "//core/model",
        "//core/model:model_data_details",
        "@com_google_absl//absl/status",
        "@flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_behavior",
    hdrs = [
        "gltf_extension_behavior.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        "//core/loader/provider/extensions/behavior:loader_extension",
        "//core/loader/provider/extensions/behavior:model_creator_extension",
        "//core/proto:json_reader",
        "@flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_behavior_impl",
    srcs = [
        "gltf_extension_behavior.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_behavior",
        "//core/loader/provider/extensions/behavior:behavior_imp_cc_proto",
        "//core/loader/provider/extensions/behavior:helpers",
        "//core/loader/provider/extensions/behavior:loader_extension",
        "//core/loader/provider/extensions/behavior:loader_extension_impl",
        "//core/loader/provider/extensions/behavior:model_creator_extension",
        "//core/loader/provider/extensions/behavior:model_creator_extension_impl",
        "//core/math",
        "//core/proto:json_reader",
        "//core/proto:proto_common",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_behavior_noop",
    srcs = [
        "gltf_extension_behavior_noop.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_behavior",
        "//core/loader/provider/extensions/behavior:behavior_imp_cc_proto",
        "//core/loader/provider/extensions/behavior:loader_extension",
        "//core/loader/provider/extensions/behavior:model_creator_extension",
        "//core/proto:json_reader",
        "@com_google_absl//absl/status:statusor",
        "@flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_interactivity",
    hdrs = [
        "gltf_extension_interactivity.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        "//core/loader/provider/extensions/interactivity:loader_extension",
        "//core/loader/provider/extensions/interactivity:model_creator_extension",
        "//core/proto:json_reader",
        "@flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_interactivity_impl",
    srcs = [
        "gltf_extension_interactivity.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_interactivity",
        "//core/loader/provider/extensions/interactivity:helpers",
        "//core/loader/provider/extensions/interactivity:interactivity_imp_cc_proto",
        "//core/loader/provider/extensions/interactivity:loader_extension",
        "//core/loader/provider/extensions/interactivity:loader_extension_impl",
        "//core/loader/provider/extensions/interactivity:model_creator_extension",
        "//core/loader/provider/extensions/interactivity:model_creator_extension_impl",
        "//core/math",
        "//core/proto:json_reader",
        "//core/proto:proto_common",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "extension_interactivity_noop",
    srcs = [
        "gltf_extension_interactivity_noop.cc",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    linkstatic = True,
    deps = [
        ":extension_interactivity",
        "//core/loader/provider/extensions/interactivity:interactivity_imp_cc_proto",
        "//core/loader/provider/extensions/interactivity:loader_extension",
        "//core/loader/provider/extensions/interactivity:model_creator_extension",
        "//core/proto:json_reader",
        "@com_google_absl//absl/status:statusor",
        "@flatbuffers",
    ],
    alwayslink = 1,
)

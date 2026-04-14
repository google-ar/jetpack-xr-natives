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
load("//build_tools:imp.bzl", "imp_copts", "imp_defines")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "base_mesh_builder",
    hdrs = [
        "base_mesh_builder.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/math",
        "@com_google_absl//absl/strings:string_view",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:filabridge_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "mesh_builder",
    srcs = [
        "mesh_builder.cc",
    ],
    hdrs = [
        "mesh_builder.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":base_mesh_builder",
        "//core/common:typed_ids",
        "//core/math",
        "//core/split_engine:split_engine_serializer",
        "//core/view:base_view",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//filament/libs:filabridge_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "mesh_data",
    srcs = [
        "mesh_data.cc",
        "mesh_data_helper.cc",
        "mesh_description.cc",
        "mesh_index_data.cc",
        "mesh_vertex_data.cc",
        "vertex_format.cc",
    ],
    hdrs = [
        "mesh_data.h",
        "mesh_data_helper.h",
        "mesh_description.h",
        "mesh_index_data.h",
        "mesh_vertex_and_index_data.h",
        "mesh_vertex_data.h",
        "vertex_format.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = 1,
    deps = [
        "//core/common:platform_helpers",
        "//core/loader:loader_details",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//filament/libs:filabridge_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "mesh",
    srcs = [
        "mesh.cc",
        "mesh_gpu_data.cc",
    ],
    hdrs = [
        "mesh.h",
        "mesh_gpu_data.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":mesh_data",
        "//core/async:executor",
        "//core/async:future",
        "//core/collision:bvh",
        "//core/common:owned_or_unowned_memory",
        "//core/common:owned_ptr",
        "//core/geometry/shapes",
        "//core/math",
        "//core/math:aabb_helpers",
        "//core/split_engine:split_engine_serializer",
        "//core/view:base_view",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "mesh_factory",
    srcs = ["mesh_factory.cc"],
    hdrs = ["mesh_factory.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":base_mesh_builder",
        ":mesh",
        ":mesh_builder",
        ":mesh_data",
        "//core/common:robin_map",
        "//core/common:typed_ids",
        "//core/geometry/shapes",
        "//core/loader:loader_options",
        "//core/math",
        "//core/math:aabb_helpers",
        "//core/view:base_view",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

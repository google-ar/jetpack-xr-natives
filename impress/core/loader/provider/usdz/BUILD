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
    name = "usdz_consumer",
    srcs = [
        "examined_model.cc",
        "mesh_geometry.cc",
        "mesh_material.cc",
        "mesh_parts.cc",
        "usdz_consumer.cc",
    ],
    hdrs = [
        "examined_model.h",
        "mesh_geometry.h",
        "mesh_material.h",
        "mesh_parts.h",
        "usdz_consumer.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/common:buffer_access",
        "//core/common:fbs",
        "//core/common:flatbuffer_helpers",
        "//core/common:log",
        "//core/common:optional_error",
        "//core/common:platform_helpers",
        "//core/common:robin_map",
        "//core/common:typed_ids",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/loader/provider:loaded_model_builder",
        "//core/loader/provider/gltf:dense_data_access",
        "//core/material_library:fbs",
        "//core/material_library:generic_material_parameters",
        "//core/material_library:generic_material_spec",
        "//core/math",
        "//core/math:flatbuffer_support",
        "//core/model:model_data_details",
        "//third_party/tinyusdz",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:geometry",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "usdz",
    srcs = [
        "provider_usdz.cc",
    ],
    hdrs = [
        "provider_usdz.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":usdz_consumer",
        "//core/common",
        "//core/common:flatbuffer_helpers",
        "//core/common:robin_map",
        "//core/common:typed_ids",
        "//core/loader/provider:details",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/model",
        "//third_party/tinyusdz",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
    ],
    alwayslink = 1,
)

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
load("//build_tools:imp.bzl", "imp_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "model",
    srcs = ["model_data.cc"],
    hdrs = [
        "model_data.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":model_data_details",
        "//core/common:filament_helpers",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/material_library:generic_material",
        "//core/material_library:material_param_value",
        "//core/math",
        "//core/render",
        "//core/split_engine:split_engine_serializer",
        "//core/view:base_view",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "model_data_details",
    hdrs = [
        "behavior_data.h",
        "entity_data.h",
        "interactivity_data.h",
        "joint_data.h",
        "shared_data.h",
        "skeleton_data.h",
        "skin_data.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        "//core/common",
        "//core/common:bit_vector",
        "//core/common:data_helpers",
        "//core/common:hash",
        "//core/common:robin_map",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/material_library:material_param_value",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "//core/render",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

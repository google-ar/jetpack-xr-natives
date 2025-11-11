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
load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "nodes_pointers",
    srcs = [
        "nodes.cc",
    ],
    hdrs = [
        "nodes.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/assets/gltf/object_model:property_pointer",
        "//core/assets/gltf/object_model:token_parser",
        "//core/math",
        "//core/model:model_data_details",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "materials_pointers",
    srcs = [
        "materials.cc",
    ],
    hdrs = [
        "materials.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/assets/gltf/object_model:property_pointer",
        "//core/assets/gltf/object_model:token_parser",
        "//core/material_library:generic_material",
        "//core/material_library:material_param_value",
        "//core/materials",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "skins_pointers",
    srcs = ["skins.cc"],
    hdrs = ["skins.h"],
    copts = imp_google3_copts(),
    deps = [
        "//core/assets/gltf/object_model:property_pointer",
        "//core/assets/gltf/object_model:token_parser",
        "//core/common:typed_ids",
        "//core/common:typed_set_vector",
        "//core/model:model_data_details",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "meshes_pointers",
    srcs = ["meshes.cc"],
    hdrs = ["meshes.h"],
    copts = imp_google3_copts(),
    deps = [
        "//core/assets/gltf/object_model:property_pointer",
        "//core/assets/gltf/object_model:token_parser",
        "//core/common:robin_map",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "core_pointers",
    srcs = [
        "core_pointers.cc",
    ],
    hdrs = [
        "core_pointers.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":materials_pointers",
        ":meshes_pointers",
        ":nodes_pointers",
        ":skins_pointers",
        "//core/assets/gltf/object_model:property_pointer",
    ],
    alwayslink = 1,
)

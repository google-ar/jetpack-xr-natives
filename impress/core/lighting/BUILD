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
load("//build_tools:imp.bzl", "imp_copts", "imp_google3_copts")
load("//core/proto:imp_cc_proto_library.bzl", "imp_cc_proto_library")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "light_component",
    srcs = [
        "light_component.cc",
    ],
    hdrs = [
        "light_component.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":light_state_proto_cc_imp",
        "//core/camera",
        "//core/math",
        "//core/ncsb",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

proto_library(
    name = "light_state_proto",
    srcs = ["light_state.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "light_state_proto_cc_imp",
    proto = ":light_state_proto",
)

cc_library(
    name = "image_based_lighting_asset",
    srcs = [
        "image_based_lighting_asset.cc",
    ],
    hdrs = [
        "image_based_lighting_asset.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":image_based_lighting_types",
        "//core/async:executor",
        "//core/async:future",
        "//core/common",
        "//core/common:buffer_access",
        "//core/common:context",
        "//core/common:invocable",
        "//core/common:small_source_location",
        "//core/common:zip_helpers",
        "//core/image",
        "//core/image:decode_image",
        "//core/render",
        "//core/resources",
        "//core/split_engine:split_engine_serializer",
        "//core/view:base_view",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)

cc_library(
    name = "image_based_lighting_asset_iblprefilter_loader",
    srcs = [
        "image_based_lighting_asset_iblprefilter_loader.cc",
    ],
    hdrs = [
        "image_based_lighting_asset_iblprefilter_loader.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image_based_lighting_asset",
        "//core/async:executor",
        "//core/common:buffer_access",
        "//core/math",
        "//core/render",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@third_party//filament/libs:iblprefilter",
        "@third_party//filament/libs:image_hdrs",
        "@third_party//filament/libs:imageio",
    ],
    alwayslink = 1,
)

cc_library(
    name = "environment_light",
    srcs = [
        "environment_light.cc",
    ],
    hdrs = [
        "environment_light.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":image_based_lighting_asset",
        "//core/assets",
        "//core/common:owned_or_borrowed_ptr",
        "//core/common:owned_or_unowned_memory",
        "//core/common:owned_ptr",
        "//core/math",
        "//core/render",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "environment_light_factory",
    srcs = [
        "environment_light_factory.cc",
    ],
    hdrs = [
        "environment_light_factory.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":environment_light",
        ":image_based_lighting_asset",
        "//core/math",
        "//core/render",
        "//core/view:base_view",
        "@com_google_absl//absl/memory",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "image_based_lighting_types",
    hdrs = ["image_based_lighting_types.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        "//core/image",
        "//core/math",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

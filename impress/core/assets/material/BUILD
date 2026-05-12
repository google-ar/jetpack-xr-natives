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
    name = "material_load_options_proto",
    srcs = ["material_load_options.proto"],
)

imp_cc_proto_library(
    name = "material_load_options_proto_cc",
    proto = ":material_load_options_proto",
    deps = [
        ":material_load_options_proto",
    ],
)

cc_library(
    name = "material_asset",
    srcs = [
        "material_asset.cc",
    ],
    hdrs = [
        "material_asset.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":material_helpers",
        ":material_load_options_proto_cc",
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:trace",
        "//core/resources",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "material_helpers",
    srcs = [
        "material_helpers.cc",
    ],
    hdrs = [
        "material_helpers.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":material_load_options_proto_cc",
        "//core/async:future",
        "@com_google_absl//absl/status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:filabridge_hdrs",
    ],
    alwayslink = 1,
)

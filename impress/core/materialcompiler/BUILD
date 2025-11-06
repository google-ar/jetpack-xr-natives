# Copyright 2025 Google LLC
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
load(
    "//build_tools:imp.bzl",
    "IMP_FLATBUFFER_INCLUDE_PATHS",
    "IMP_FLATC_ARGS",
    "if_non_android",
    "imp_copts",
)
load("//third_party/flatbuffers:flatbuffers.bzl", "flatbuffers_library")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

flatbuffer_cc_library(
    name = "fbs",
    srcs = ["schemas/material_compiler_ipc.fbs"],
    flatc_args = IMP_FLATC_ARGS,
    include_paths = IMP_FLATBUFFER_INCLUDE_PATHS,
    includes = [
    ],
    out_prefix = "schemas/",
)

flatbuffers_library(
    name = "schemas/material_compiler_ipc_fbs",
    srcs = ["schemas/material_compiler_ipc.fbs"],
)

cc_library(
    name = "runtime_material_compiler_config",
    hdrs = ["runtime_material_compiler_config.h"],
    copts = imp_copts(),
    deps = [
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:matp",
    ],
    alwayslink = 1,
)

cc_library(
    name = "material_compiler_client",
    srcs = ["material_compiler_client.cc"],
    hdrs = ["material_compiler_client.h"],
    copts = imp_copts(),
    deps = [
        ":fbs",
        "//core/common:buffer_access",
        "//core/common:flatbuffer_helpers",
        "//core/ipc:common",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "material_compiler_service",
    srcs = ["material_compiler_service.cc"],
    hdrs = ["material_compiler_service.h"],
    copts = imp_copts(),
    deps = [
        ":fbs",
        ":runtime_material_compiler_config",
        "//core/ipc:common",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/libs:filamat",
        "@third_party//filament/libs:matp",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "material_compiler_client_jni",
    srcs = ["material_compiler_client_jni.cc"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":material_compiler_client",
        "//core/async:future",
        "//core/common:context",
        "//core/common:jni_helpers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "material_compiler_service_jni",
    srcs = ["material_compiler_service_jni.cc"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":material_compiler_service",
        "//core/common:jni_helpers",
        "//core/common:platform_helpers",
        "@com_google_absl//absl/memory",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "test_material_compiler_service",
    testonly = True,
    hdrs = ["test_material_compiler_service.h"],
    copts = imp_copts(),
    deps = [
        ":fbs",
        "//core/ipc:common",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:gtest",
    ],
    alwayslink = 1,
)

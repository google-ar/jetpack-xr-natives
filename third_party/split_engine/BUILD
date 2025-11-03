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
    "@com_google_impress//build_tools:imp.bzl",
    "IMP_FLATBUFFER_INCLUDE_PATHS",
    "IMP_FLATC_ARGS",
    "if_non_android",
)
load(
    "@flatbuffers//:build_defs.bzl",
    "flatbuffer_library_public",
)
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_license//rules:license_kind.bzl", "license_kind")
load("@third_party//split_engine/build_tools:gen_max_api_requirement.bzl", "gen_max_api_requirement")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

exports_files([
    "LICENSE",
])

license_kind(
    name = "license",
    conditions = [
        "notice",
    ],
)

schemas = [
    "schemas/split_engine_attributes.fbs",
    "schemas/split_engine_data.fbs",
    "schemas/split_engine_ipc.fbs",
    "schemas/split_engine_material.fbs",
    "schemas/split_engine_primitive.fbs",
]

filegroup(
    name = "schemas",
    srcs = schemas,
)

flatbuffer_library_public(
    name = "fbs_srcs",
    srcs = schemas,
    outs = [
        item
        for f in schemas
        for item in (
            "schemas/" + f.replace(".fbs", "_generated.h").split("/")[-1],
            "schemas/" + f.replace(".fbs", "_bfbs_generated.h").split("/")[-1],
        )
    ],
    flatc_args = IMP_FLATC_ARGS + [
        "--bfbs-gen-embed",  # Enable embedding of the flatbuffers schema in the binary.
        "--gen-name-strings",  # Generate fully qualified names for the flatbuffers tables to find them in the embedded schema.
    ],
    include_paths = IMP_FLATBUFFER_INCLUDE_PATHS,
    language_flag = "-c",
    out_prefix = "schemas/",
)

cc_library(
    name = "fbs",
    srcs = [":fbs_srcs"],
    hdrs = [":fbs_srcs"],
    deps = [
        "@flatbuffers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "fbs_deps",
    deps = [
        ":fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_schema_version",
    hdrs = ["schemas/split_engine_schema_version.h"],
    deps = [
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

gen_max_api_requirement(
    name = "max_api_requirement",
    srcs = schemas,
    out = "schemas/max_api_requirement.h",
)

cc_library(
    name = "subspace_root",
    srcs = ["subspace_root.cc"],
    hdrs = ["subspace_root.h"],
    deps = [
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
        "@com_google_impress//core/common:rememberer",
        "@com_google_impress//core/math",
        "@com_google_impress//core/ncsb",
        "@com_google_impress//core/view:base_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_subspace_manager",
    hdrs = ["split_engine_subspace_manager.h"],
    deps = [
        ":subspace_root",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/math",
        "@third_party//split_engine/input",
        "@third_party//split_engine/input:split_engine_input_event",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_subspace_manager_impl",
    srcs = ["split_engine_subspace_manager_impl.cc"],
    hdrs = ["split_engine_subspace_manager_impl.h"],
    linkstatic = True,
    deps = [
        ":split_engine_subspace_manager",
        ":subspace_events",
        ":subspace_root",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/async:executor",
        "@com_google_impress//core/async:future",
        "@com_google_impress//core/common:log",
        "@com_google_impress//core/common:rememberer",
        "@com_google_impress//core/math",
        "@com_google_impress//core/ncsb",
        "@com_google_impress//core/view:base_view",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine/input",
        "@third_party//split_engine/input:split_engine_input_event",
    ],
    alwayslink = 1,
)

cc_library(
    name = "subspace_events",
    hdrs = ["subspace_events.h"],
    deps = [
        ":subspace_root",
        "@com_google_impress//core/ncsb",
    ],
    alwayslink = 1,
)

cc_library(
    name = "subspace_jni",
    srcs = [
        "subspace_jni.cc",
    ],
    deps = [
        ":split_engine_subspace_manager_impl",
        ":subspace_root",
        "@com_google_impress//core/common:jni_helpers",
        "@com_google_impress//core/common:log",
        "@com_google_impress//core/math",
        "@com_google_impress//core/view:base_view",
        "@com_google_impress//core/view:view_host",
        "@third_party//split_engine/input",
        "@third_party//split_engine/input:split_engine_input_event",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

# Stops build_cleaner from generating flatbuffers_library for the split engine schemas.
filegroup(
    name = "build_cleaner_ignore",
    srcs = [
        "schemas/split_engine_data.fbs",
        "schemas/split_engine_ipc.fbs",
    ],
    tags = ["ignore_srcs"],
)

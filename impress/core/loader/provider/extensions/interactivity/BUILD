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

load(
    "@com_github_google_flatbuffers//:build_defs.bzl",
    "flatbuffer_cc_library",
)
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "IMP_FLATBUFFER_INCLUDE_PATHS", "IMP_FLATC_ARGS", "imp_copts", "imp_linkopts")
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

schemas = [
    "schemas/interactivity.fbs",
]

filegroup(
    name = "schemas",
    srcs = schemas,
)

flatbuffer_cc_library(
    name = "fbs",
    srcs = schemas,
    flatc_args = IMP_FLATC_ARGS,
    include_paths = IMP_FLATBUFFER_INCLUDE_PATHS,
    includes = [
        "//core/common:schemas",
    ],
    out_prefix = "schemas/",
)

cc_library(
    name = "fbs_deps",
    deps = [
        ":fbs",
        "//core/common:fbs_deps",
    ],
    alwayslink = 1,
)

proto_library(
    name = "interactivity_proto",
    srcs = ["interactivity.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "interactivity_imp_cc_proto",
    proto = ":interactivity_proto",
    deps = [
        "//core/math:math_proto_cc",
    ],
)

cc_library(
    name = "helpers",
    srcs = [
        "interactivity_helpers.cc",
    ],
    hdrs = [
        "interactivity_helpers.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    deps = [
        ":interactivity_imp_cc_proto",
        "//core/math",
    ],
    alwayslink = 1,
)

cc_library(
    name = "loader_extension",
    hdrs = [
        "loader_extension.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    deps = [
        ":fbs",
        ":fbs_deps",  # build_cleaner: keep
        ":interactivity_imp_cc_proto",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "loader_extension_impl",
    srcs = [
        "loader_extension_impl.cc",
    ],
    hdrs = [
        "loader_extension_impl.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    deps = [
        ":fbs",
        ":fbs_deps",  # build_cleaner: keep
        ":helpers",
        ":interactivity_imp_cc_proto",
        ":loader_extension",
        "//core/common:fbs",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/common:flatbuffer_helpers",
        "//core/common:typed_ids",
        "//core/math",
        "//core/math:flatbuffer_support",
        "//core/model",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "model_creator_extension",
    hdrs = [
        "model_creator_extension.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    deps = [
        ":fbs",
        ":fbs_deps",  # build_cleaner: keep
        "//core/model",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "model_creator_extension_impl",
    srcs = [
        "model_creator_extension_impl.cc",
    ],
    hdrs = [
        "model_creator_extension_impl.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    deps = [
        ":fbs",
        ":fbs_deps",  # build_cleaner: keep
        ":model_creator_extension",
        "//core/math",
        "//core/math:flatbuffer_support",
        "//core/model",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

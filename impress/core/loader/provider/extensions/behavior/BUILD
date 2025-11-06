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
    name = "behavior_proto",
    srcs = ["behavior.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "behavior_imp_cc_proto",
    proto = ":behavior_proto",
    deps = [
        "//core/math:math_proto_cc",
    ],
)

cc_library(
    name = "helpers",
    srcs = [
        "behavior_helpers.cc",
    ],
    hdrs = [
        "behavior_helpers.h",
    ],
    copts = imp_copts(),
    linkopts = imp_linkopts(),
    deps = [
        ":behavior_imp_cc_proto",
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
        ":behavior_imp_cc_proto",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "@com_google_absl//absl/status:statusor",
        "@flatbuffers",
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
        ":behavior_imp_cc_proto",
        ":helpers",
        ":loader_extension",
        "//core/common:fbs",
        "//core/common:fbs_deps",  # build_cleaner: keep
        "//core/common:flatbuffer_helpers",
        "//core/common:typed_ids",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//core/math",
        "//core/math:flatbuffer_support",
        "//core/model",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:variant",
        "@flatbuffers",
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
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
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
        ":model_creator_extension",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
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

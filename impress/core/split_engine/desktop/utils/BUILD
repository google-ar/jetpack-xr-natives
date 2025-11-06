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

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load(
    "//build_tools:imp.bzl",
    "imp_copts",
    "imp_defines",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "split_engine_desktop_bridge_uds",
    hdrs = ["split_engine_desktop_bridge_uds.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = ["@com_google_absl//absl/log:check"],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_desktop_bridge_utils",
    srcs = [
        "split_engine_desktop_bridge_fdr.cc",
        "split_engine_desktop_bridge_fds.cc",
    ],
    hdrs = [
        "split_engine_desktop_bridge_epoll.h",
        "split_engine_desktop_bridge_utils.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_desktop_bridge_uds",
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/functional:any_invocable",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_desktop_bridge_heartbeat",
    srcs = ["split_engine_desktop_bridge_heartbeat.cc"],
    hdrs = ["split_engine_desktop_bridge_heartbeat.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "message_group_completion_client_reactor",
    srcs = ["message_group_completion_client_reactor.cc"],
    hdrs = ["message_group_completion_client_reactor.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/desktop:split_engine_desktop_bridge_cc_proto",
        "//third_party/grpc:grpc++_unsecure",
        "@com_google_absl//absl/log:check",
    ],
    alwayslink = 1,
)

cc_library(
    name = "message_group_completion_server_reactor",
    srcs = ["message_group_completion_server_reactor.cc"],
    hdrs = ["message_group_completion_server_reactor.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/desktop:split_engine_desktop_bridge_cc_proto",
        "//third_party/grpc:grpc++_unsecure",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/synchronization",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "buffer_factory",
    srcs = ["buffer_factory.cc"],
    hdrs = ["buffer_factory.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/numeric:int128",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "buffer_factory_heap",
    srcs = ["buffer_factory_heap.cc"],
    hdrs = ["buffer_factory_heap.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":buffer_factory",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/numeric:int128",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
    ],
    alwayslink = 1,
)

cc_library(
    name = "buffer_factory_shmem",
    srcs = ["buffer_factory_shmem.cc"],
    hdrs = ["buffer_factory_shmem.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":buffer_factory",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/numeric:int128",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

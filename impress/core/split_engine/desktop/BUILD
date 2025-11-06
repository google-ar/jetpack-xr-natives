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
load("//net/grpc:cc_grpc_library.bzl", "cc_grpc_library")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

proto_library(
    name = "split_engine_desktop_bridge_proto",
    srcs = ["split_engine_desktop_bridge.proto"],
    has_services = True,
    deps = ["//google/rpc:status"],
)

cc_proto_library(
    name = "split_engine_desktop_bridge_cc_proto",
    deps = [":split_engine_desktop_bridge_proto"],
)

cc_grpc_library(
    name = "split_engine_desktop_bridge_cc_grpc",
    srcs = [":split_engine_desktop_bridge_proto"],
    deps = [":split_engine_desktop_bridge_cc_proto"],
)

cc_library(
    name = "split_engine_desktop_bridge_service_impl",
    srcs = ["split_engine_desktop_bridge_service_impl.cc"],
    hdrs = ["split_engine_desktop_bridge_service_impl.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_desktop_bridge_cc_grpc",  # buildcleaner: keep
        "//core/async:executor",
        "//core/common:enum_flags",
        "//core/common:registry",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android:split_engine_shared_memory_bridge_service_impl",
        "//core/split_engine/desktop/utils:message_group_completion_server_reactor",
        "//core/split_engine/desktop/utils:split_engine_desktop_bridge_heartbeat",
        "//core/split_engine/desktop/utils:split_engine_desktop_bridge_utils",
        "//core/view:base_view",
        "//third_party/grpc:grpc++",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_desktop_bridge_client",
    srcs = ["split_engine_desktop_bridge_client.cc"],
    hdrs = ["split_engine_desktop_bridge_client.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_desktop_bridge_cc_grpc",
        ":split_engine_desktop_bridge_cc_proto",
        "//core/split_engine:split_engine_bridge_sender",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android:split_engine_shared_memory_bridge_client",
        "//core/split_engine/desktop/utils:message_group_completion_client_reactor",
        "//core/split_engine/desktop/utils:split_engine_desktop_bridge_utils",
        "//third_party/grpc:grpc++_unsecure",
        "@bazel_tools//tools/jdk:jni",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

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

cc_library(
    name = "split_engine_desktop_bridge_sender",
    srcs = ["split_engine_desktop_bridge_sender.cc"],
    hdrs = ["split_engine_desktop_bridge_sender.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_desktop_bridge_client",
        "//core/split_engine:flatbuffer_allocator",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android:bridge_buffer",
        "//core/split_engine/android:buffer_handle_factory",
        "//core/split_engine/android:split_engine_shared_memory_bridge_sender_base",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_desktop_bridge_client",
    hdrs = ["split_engine_desktop_bridge_client.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_desktop_bridge_client_impl",
    srcs = ["split_engine_desktop_bridge_client_impl.cc"],
    hdrs = ["split_engine_desktop_bridge_client_impl.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_desktop_bridge_cc_grpc",  # buildcleaner: keep
        ":split_engine_desktop_bridge_client",
        "//core/split_engine:split_engine_bridge_sender",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/desktop/utils:message_group_completion_client_reactor",
        "//third_party/grpc:grpc++_unsecure",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

proto_library(
    name = "split_engine_desktop_bridge_proto",
    srcs = ["split_engine_desktop_bridge.proto"],
    has_services = True,
    deps = [
        "//core/split_engine/desktop:split_engine_desktop_bridge_proto",
        "//google/rpc:status",
    ],
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
    name = "split_engine_desktop_bridge_service",
    srcs = ["split_engine_desktop_bridge_service.cc"],
    hdrs = ["split_engine_desktop_bridge_service.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_desktop_bridge_cc_grpc",  # buildcleaner: keep
        ":split_engine_desktop_bridge_service_impl_facade",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/desktop/utils:message_group_completion_server_reactor",
        "//core/split_engine/desktop/utils:split_engine_desktop_bridge_heartbeat",
        "//third_party/grpc:grpc++_unsecure",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_desktop_bridge_service_impl_facade",
    hdrs = ["split_engine_desktop_bridge_service_impl_facade.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_desktop_bridge_service_impl_local",
    srcs = ["split_engine_desktop_bridge_service_impl_local.cc"],
    hdrs = ["split_engine_desktop_bridge_service_impl_local.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_desktop_bridge_service_impl_facade",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:enum_flags",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/desktop/utils:buffer_factory",
        "//core/split_engine/desktop/utils:buffer_factory_heap",
        "//core/view:base_view",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_desktop_bridge_service_impl_proxy",
    srcs = ["split_engine_desktop_bridge_service_impl_proxy.cc"],
    hdrs = ["split_engine_desktop_bridge_service_impl_proxy.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_desktop_bridge_service_impl_facade",
        "//core/split_engine:split_engine_bridge_sender",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android:split_engine_android_bridge",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_desktop_bridge_legacy",
    srcs = ["split_engine_desktop_bridge_legacy.cc"],
    hdrs = ["split_engine_desktop_bridge_legacy.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_desktop_bridge_client",
        "//core/split_engine/android:split_engine_android_bridge",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

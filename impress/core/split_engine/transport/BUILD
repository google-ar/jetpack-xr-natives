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
    name = "transport",
    hdrs = ["transport.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/common:invocable",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "transport_with_hooks",
    srcs = ["transport_with_hooks.cc"],
    hdrs = ["transport_with_hooks.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":transport",
        "//core/common:owned_or_borrowed_ptr",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "basic_transport",
    hdrs = ["basic_transport.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":transport",
        ":transport_session_memory_manager",
        "//core/async:background_scheduler",
        "//core/common:owned_ptr",
        "//core/split_engine/transport/concepts",
        "//core/split_engine/transport/concepts:underlying_transport",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/numeric:int128",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "basic_transport_shmem",
    hdrs = ["basic_transport_shmem.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":basic_transport",
        ":shared_memory_region_allocator",
        ":transport",
        ":transport_session_memory_manager_impl",
        "//core/async:background_scheduler",
        "//core/common:owned_ptr",
        "//core/split_engine:flatbuffer_allocator",
        "//core/split_engine/transport/concepts",
        "//core/split_engine/transport/concepts:shared_memory_transport",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "basic_transport_request_adapter",
    hdrs = ["basic_transport_request_adapter.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":transport",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_binder_transport",
    hdrs = ["split_engine_binder_transport.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":basic_transport_shmem",
        "//core/split_engine/android/extensions:split_engine_bridge",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_grpc_transport_sm",
    hdrs = ["split_engine_grpc_transport_sm.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":basic_transport_shmem",
        "//core/split_engine/desktop:split_engine_desktop_bridge_client",
    ],
    alwayslink = 1,
)

cc_library(
    name = "transport_session_memory_manager",
    hdrs = ["transport_session_memory_manager.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    alwayslink = 1,
)

cc_library(
    name = "transport_session_memory_manager_impl",
    hdrs = ["transport_session_memory_manager_impl.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":transport_session_memory_manager",
        "//core/common:owned_ptr",
        "//core/split_engine:flatbuffer_allocator",
        "@com_google_absl//absl/log:check",
    ],
    alwayslink = 1,
)

cc_library(
    name = "shared_memory_region_allocator",
    hdrs = ["shared_memory_region_allocator.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":transport_session_memory_manager_impl",
        "//core/async:background_scheduler",
        "//core/common:owned_ptr",
        "//core/split_engine/android:bridge_buffer",
        "//core/split_engine/android:buffer_handle_factory",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/synchronization",
    ],
    alwayslink = 1,
)

test_suite(
    name = "all_tests",
    tests = [
        ":basic_transport_test",
    ],
)

cc_library(
    name = "senders",
    srcs = [
        "message_sender.cc",
        "request_sender.cc",
    ],
    hdrs = [
        "flatbuffer_builder_holder.h",
        "message_sender.h",
        "request_sender.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":transport",
        "//core/async:background_scheduler",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:invocable",
        "//core/common:owned_or_borrowed_ptr",
        "//core/common:owned_ptr",
        "//core/common:pass_key",
        "//core/split_engine:flatbuffer_utils",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

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

# Shared memory implementation of Impress Split Engine bridge, targeted for Android.

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load(
    "//build_tools:imp.bzl",
    "if_android",
    "if_non_android",
    "imp_copts",
    "imp_defines",
    "imp_test_copts",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "split_engine_android_bridge",
    hdrs = [
        "split_engine_android_bridge.h",
    ],
    deps = [
        ":split_engine_shared_memory_bridge_client",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:invocable",
        "//core/math:flatbuffer_support",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "split_engine_android_bridge_mock",
    testonly = True,
    hdrs = [
        "split_engine_android_bridge_mock.h",
    ],
    copts = imp_test_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_android_bridge",
        ":split_engine_shared_memory_bridge_client",
        "//core/common:invocable",
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:gtest",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "split_engine_android_shared_memory_bridge",
    srcs = ["split_engine_android_shared_memory_bridge.cc"],
    hdrs = [
        "split_engine_android_shared_memory_bridge.h",
    ],
    deps = [
        ":split_engine_android_bridge",
        ":split_engine_shared_memory_bridge_client",
        "//core/common:invocable",
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

# Requires: --config=android_arm --android_ndk_min_sdk_version=34
cc_library(
    name = "split_engine_shared_memory_bridge_client",
    hdrs = [
        "split_engine_shared_memory_bridge_client.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    # TODO: (broken link) - These AIDL builds are not compatible with Tricorder for some reason.
    tags = [
        "manual",
        "nobuilder",
        "notap",
    ],
    deps = [
        ":buffer_handle_factory",
        "//core/common:invocable",
        "//core/math",
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "split_engine_shared_memory_bridge_client_mock",
    testonly = True,
    hdrs = [
        "split_engine_shared_memory_bridge_client_mock.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":buffer_handle_factory",
        ":split_engine_shared_memory_bridge_client",
        "//core/common:invocable",
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:gtest",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

# Requires: --config=android_arm --android_ndk_min_sdk_version=34
cc_library(
    name = "split_engine_shared_memory_bridge_client_ndk",
    srcs = [
        "split_engine_shared_memory_bridge_client_ndk.cc",
    ],
    hdrs = [
        "split_engine_shared_memory_bridge_client_ndk.h",
    ],
    copts = imp_copts() + if_android(["-Wno-private-header"]),
    defines = imp_defines(),
    linkopts = [
        "-lbinder_ndk",
    ],
    # TODO: (broken link) - These AIDL builds are not compatible with Tricorder for some reason.
    tags = [
        "manual",
        "nobuilder",
        "notap",
    ],
    deps = [
        ":split_engine_shared_memory_bridge_client",
        "//core/common:invocable",
        "//core/common:trace",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android/aidl:split_engine_shared_memory_bridge_service_aidl_ndk_lib",
        "//core/split_engine/android/extensions:split_engine_bridge",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_shared_memory_bridge_sender_base",
    srcs = ["split_engine_shared_memory_bridge_sender_base.cc"],
    hdrs = ["split_engine_shared_memory_bridge_sender_base.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":bridge_buffer",
        ":buffer_handle_factory",
        "//core/async:background_scheduler_impl",
        "//core/async:executor",
        "//core/common:invocable",
        "//core/common:owned_ptr",
        "//core/common:robin_map",
        "//core/split_engine:flatbuffer_allocator",
        "//core/split_engine:flatbuffer_size_calculator",
        "//core/split_engine:message_group_monitor",
        "//core/split_engine:split_engine_bridge_sender",
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "bridge_buffer",
    srcs = [
        "bridge_buffer.cc",
    ],
    hdrs = [
        "bridge_buffer.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":buffer_handle_factory",
        "//core/async:background_scheduler",
        "//core/async:executor",
        "//core/common:trace",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
        "@com_google_impress//core/common:log",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

# Requires: --config=android_arm --android_ndk_min_sdk_version=34
cc_library(
    name = "split_engine_shared_memory_bridge_sender",
    srcs = [
        "split_engine_shared_memory_bridge_sender.cc",
    ],
    hdrs = [
        "split_engine_shared_memory_bridge_sender.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    # TODO: (broken link) - These AIDL builds are not compatible with Tricorder for some reason.
    tags = [
        "manual",
        "nobuilder",
        "notap",
    ],
    deps = [
        ":bridge_buffer",
        ":buffer_handle_factory",
        ":buffer_handle_factory_shmem",
        ":split_engine_shared_memory_bridge_client",
        ":split_engine_shared_memory_bridge_sender_base",
        "//core/async:executor",
        "//core/common:owned_ptr",
        "//core/split_engine:flatbuffer_allocator",
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//split_engine:fbs",  # buildcleaner: keep
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

# Requires: --config=android_arm --android_ndk_min_sdk_version=34

# Requires: --config=android_arm --android_ndk_min_sdk_version=34
cc_library(
    name = "split_engine_shared_memory_bridge_service",
    srcs = [
        "split_engine_shared_memory_bridge_service.cc",
    ],
    hdrs = [
        "split_engine_shared_memory_bridge_service.h",
    ],
    copts = if_android(["-Wno-private-header"]),
    deps = [
        ":split_engine_shared_memory_bridge_service_impl",
        "//core/async:executor",
        "//core/common:enum_flags",
        "//core/common:registry",
        "//core/common:trace",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android/aidl:split_engine_shared_memory_bridge_service_aidl_ndk_lib",
        "//core/split_engine/android/aidl:split_engine_shared_memory_bridge_token_aidl_ndk_lib",
        "//core/view:base_view",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_android_surface_factory",
    srcs = [
        "split_engine_android_surface_factory.cc",
    ],
    hdrs = [
        "split_engine_android_surface_factory.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core:config",
        "//core/common:invocable",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/math",
        "//core/media:media_color_space",
        "//core/render",
        "//core/render:content_security_level",
        "//core/render:display_color_space",
        "//core/render/android:android_external_texture_surface",
        "//core/render/android:definitions",
        "//core/render/android:platform_android_external_texture_surface",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]) + if_android([
        "//core/view/platforms/xr_android:xr",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "split_engine_platform_android_external_texture_surface",
    srcs = ["split_engine_platform_android_external_texture_surface.cc"],
    hdrs = ["split_engine_platform_android_external_texture_surface.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":split_engine_android_bridge",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media:media_color_space",
        "//core/render",
        "//core/render:content_security_level",
        "//core/render/android:definitions",
        "//core/render/android:platform_android_external_texture_surface",
        "//core/split_engine:split_engine_serializer",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "buffer_handle_factory",
    srcs = ["buffer_handle_factory.cc"],
    hdrs = ["buffer_handle_factory.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    alwayslink = 1,
)

cc_library(
    name = "shmem_utils",
    srcs = ["shmem_utils.cc"],
    hdrs = ["shmem_utils.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = if_android([
    ]) + [
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "buffer_handle_factory_shmem",
    hdrs = ["buffer_handle_factory_shmem.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":buffer_handle_factory",
        ":split_engine_shared_memory_bridge_client",
        "@com_google_absl//absl/status:statusor",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

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

load(
    "//build_tools:imp.bzl",
    "if_android",
    "if_non_android",
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
    name = "split_engine_android_bridge",
    hdrs = [
        "split_engine_android_bridge.h",
    ],
    deps = [
        ":split_engine_shared_memory_bridge_client",
        "//core/async:executor",
        "//core/async:future",
        "//core/math:flatbuffer_support",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_shared",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
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
        "//core/split_engine:split_engine_shared",
    ],
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
        "//core/math",
        "//core/split_engine:split_engine_shared",
        "@com_google_absl//absl/status:statusor",
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
        ":split_engine_shared_memory_bridge_client",
        "//core/split_engine:split_engine_shared",
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
    copts = imp_copts(),
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
        "//core/common:log",
        "//core/common:trace",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android/aidl:split_engine_shared_memory_bridge_service_aidl_ndk_lib",
    ],
    alwayslink = 1,
)

# Requires: --config=android_arm --android_ndk_min_sdk_version=34
cc_library(
    name = "split_engine_shared_memory_bridge_sender",
    srcs = [
        "bridge_buffer.cc",
        "split_engine_shared_memory_bridge_sender.cc",
    ],
    hdrs = [
        "bridge_buffer.h",
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
        ":message_group_id_mapper",
        ":split_engine_shared_memory_bridge_client",
        "//core/common:log",
        "//core/common:platform_helpers",
        "//core/common:robin_map",
        "//core/common:trace",
        "//core/split_engine:flatbuffer_allocator",
        "//core/split_engine:flatbuffer_size_calculator",
        "//core/split_engine:split_engine_bridge_sender",
        "//core/split_engine:split_engine_shared",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/functional:bind_front",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine:fbs",  # buildcleaner: keep
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

# Requires: --config=android_arm --android_ndk_min_sdk_version=34
cc_library(
    name = "split_engine_shared_memory_bridge_service_impl",
    srcs = [
        "split_engine_shared_memory_bridge_service_impl.cc",
    ],
    hdrs = [
        "split_engine_shared_memory_bridge_service_impl.h",
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
        ":split_engine_android_surface_factory",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:log",
        "//core/common:registry",
        "//core/common:rememberer",
        "//core/math",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
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
    deps = [
        ":split_engine_shared_memory_bridge_service_impl",
        "//core/async:executor",
        "//core/common:enum_flags",
        "//core/common:log",
        "//core/common:registry",
        "//core/common:trace",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android/aidl:split_engine_shared_memory_bridge_service_aidl_ndk_lib",
        "//core/split_engine/android/aidl:split_engine_shared_memory_bridge_token_aidl_ndk_lib",
        "//core/view:base_view",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
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
        "//core/common:log",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/math",
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
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]) + if_android([
        "//core/view/platforms/xr_android:xr",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "message_group_id_mapper",
    hdrs = ["message_group_id_mapper.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        "//core/split_engine:flatbuffer_allocator",
        "//core/split_engine:split_engine_shared",
    ],
    alwayslink = 1,
)

cc_library(
    name = "message_group_id_mapper_impl",
    srcs = ["message_group_id_mapper_impl.cc"],
    hdrs = ["message_group_id_mapper_impl.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":message_group_id_mapper",
        "//core/split_engine:flatbuffer_allocator",
        "//core/split_engine:split_engine_shared",
    ],
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
        "//core/common:log",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/math",
        "//core/render",
        "//core/render:content_security_level",
        "//core/render/android:definitions",
        "//core/render/android:platform_android_external_texture_surface",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_serializer",
        "//core/split_engine:split_engine_shared",
        "//core/view:base_view",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:inlined_vector",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

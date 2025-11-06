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

# IPC library for loader, used to sandbox loading from untrusted sources.

load(
    "@flatbuffers//:build_defs.bzl",
    "flatbuffer_cc_library",
)
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load(
    "//build_tools:imp.bzl",
    "IMP_FLATBUFFER_INCLUDE_PATHS",
    "IMP_FLATC_ARGS",
    "if_android",
    "if_non_android",
    "imp_copts",
    "imp_google3_copts",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

flatbuffer_cc_library(
    name = "fbs",
    srcs = ["schemas/loader_ipc.fbs"],
    flatc_args = IMP_FLATC_ARGS,
    include_paths = IMP_FLATBUFFER_INCLUDE_PATHS,
    includes = [
        "//core/animation:schemas",
        "//core/common:schemas",
        "//core/loader/provider:loader_schemas",
        "//core/loader/provider/extensions/interactivity:schemas",
        "//core/material_library:schemas",
    ],
    out_prefix = "schemas/",
)

cc_library(
    name = "fbs_deps",
    deps = [
        ":fbs",
        "//core/animation:fbs_deps",
        "//core/common:fbs_deps",
        "//core/loader/provider:fbs_deps",
        "//core/loader/provider/extensions/interactivity:fbs_deps",
        "//core/material_library:fbs_deps",
    ],
    alwayslink = 1,
)

# Wraps the java side loader in native code.
cc_library(
    name = "java_loader_client_jni",
    srcs = [
        "java_loader_client_jni.cc",
    ],
    hdrs = [
        "java_loader_client_jni.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":loader_client_base",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/ipc:common",
        "@com_google_absl//absl/memory",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "loader_client_base",
    hdrs = [
        "loader_client_base.h",
    ],
    copts = imp_copts(),
    deps = [
        ":fbs",
        "//core/common",
        "//core/common:flatbuffer_helpers",
        "//core/loader:loader_options",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "client",
    srcs = [
        "loader_client.cc",
    ],
    hdrs = [
        "loader_client.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":fbs",
        ":loader_client_base",
        "//core/common",
        "//core/common:flatbuffer_helpers",
        "//core/ipc:common",
        "//core/loader:loader_options",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "//third_party/arcore/ar/sceneform/converter/data:embedded_generic_opengl_material_data",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/libs:utils_hdrs",
    ] + if_android([
        ":java_loader_client_jni",
    ]),
    alwayslink = 1,
)

# For the sandboxed loader, the client should not link against the provider.

cc_library(
    name = "service",
    srcs = [
        "loader_service.cc",
    ],
    hdrs = [
        "loader_service.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":fbs",
        "//core/common",
        "//core/common:flatbuffer_helpers",
        "//core/ipc:common",
        "//core/loader:loader_options",
        "//core/loader/data:embedded_placeholder_textures",
        "//core/loader/provider",
        "//core/loader/provider:fbs",
        "//core/loader/provider:fbs_deps",  # build_cleaner: keep
        "@com_google_absl//absl/log:globals",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "test_utils",
    testonly = 1,
    srcs = [
        "ipc_test_helpers.cc",
    ],
    hdrs = [
        "ipc_test_helpers.h",
        "mock_loader_service.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":fbs",
        ":fbs_deps",  # build_cleaner: keep
        "//core/common",
        "//core/ipc:common",
        "//devtools/build/runtime:get_runfiles_dir",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "loader_service_jni",
    srcs = [
        "loader_service_jni/sceneform_loader_service_jni.cc",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":service",
        "//core/common:jni_helpers",
        "@com_google_absl//absl/memory",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "imp_loader_service_jni",
    srcs = [
        "loader_service_jni/imp_loader_service_jni.cc",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":service",
        "//core/common:jni_helpers",
        "@com_google_absl//absl/memory",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

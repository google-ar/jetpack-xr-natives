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
    "if_non_android",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "remote_editor_jni",
    srcs = ["remote_editor_jni.cc"],
    linkstatic = True,
    deps = [
        "//core/async:executor",
        "//core/common:buffer_access",
        "//core/common:jni_helpers",
        "//core/proto:proto_reader",
        "//core/scripting:message_helpers",
        "//core/scripting/proto:bridge_cc_proto",
        "//core/view:base_view",
        "//core/view/scripting",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "remote_editor",
    srcs = ["remote_editor_server.cc"],
    hdrs = ["remote_editor_server.h"],
    deps = [
        ":remote_editor_jni",  # buildcleaner: keep
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:jni_helpers",
        "//core/common:platform_helpers",
        "//core/ncsb",
        "//core/proto:proto_reader",
        "//core/proto:proto_writer",
        "//core/scripting/proto:bridge_cc_proto",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

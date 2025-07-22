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

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "extensions",
    deps = [
        ":split_engine_bridge",
        ":split_engine_bridge_jni",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_bridge",
    srcs = ["split_engine_bridge.cc"],
    hdrs = ["split_engine_bridge.h"],
    deps = [
        "//core/async:executor",
        "//core/common:jni_helpers",
        "//core/common:log",
        "//core/split_engine:split_engine_bridge_sender",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android:split_engine_shared_memory_bridge_client",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_bridge_jni",
    srcs = ["split_engine_bridge_jni.cc"],
    features = ["-layering_check"],
    deps = [
        ":split_engine_bridge",
        "//:platform_android",
        "//core/common:jni_helpers",
        "//core/common:log",
        "//core/split_engine:split_engine_bridge_sender",
        "//core/split_engine:split_engine_shared",
        "//core/split_engine/android:split_engine_shared_memory_bridge_client",
        "//core/view/platforms/android/jni_helpers:exception_helper",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
    ],
    alwayslink = 1,
)

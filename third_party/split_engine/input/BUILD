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

load("@com_google_impress//build_tools:imp.bzl", "if_non_android")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "input",
    deps = [
        ":split_engine_input_event",
        ":split_engine_input_event_hit_info",
        ":split_engine_input_event_jni",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_input_event",
    srcs = ["split_engine_input_event.cc"],
    hdrs = ["split_engine_input_event.h"],
    deps = [
        ":split_engine_input_event_hit_info",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/math",
        "@com_google_impress//core/ncsb",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_input_event_hit_info",
    srcs = ["split_engine_input_event_hit_info.cc"],
    hdrs = ["split_engine_input_event_hit_info.h"],
    deps = [
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/math",
        "@com_google_impress//core/ncsb",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_input_event_jni",
    srcs = ["split_engine_input_event_jni.cc"],
    deps = [
        ":split_engine_input_event",
        ":split_engine_input_event_hit_info",
        "@com_google_impress//core/common:jni_helpers",
        "@com_google_impress//core/common:log",
        "@com_google_impress//core/ncsb",
        "@third_party//filament/libs:utils_hdrs",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    # JNI needs alwayslink. The methods in this target are called from Java, on
    # the other side of the JNI boundary, rather than from static callsites in
    # C++. See:
    # (broken link)
    alwayslink = 1,
)

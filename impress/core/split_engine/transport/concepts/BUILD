# Copyright 2026 Google LLC
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

cc_library(
    name = "concepts",
    hdrs = ["concepts.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    alwayslink = 1,
)

cc_library(
    name = "underlying_transport",
    hdrs = ["underlying_transport.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    alwayslink = 1,
)

cc_library(
    name = "shared_memory_transport",
    hdrs = ["shared_memory_transport.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":underlying_transport",
        "//core/common:invocable",
        "//core/split_engine/android:buffer_handle_factory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

licenses(["notice"])

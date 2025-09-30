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

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "pointer_parser",
    srcs = [
        "pointer_parser.cc",
    ],
    hdrs = [
        "pointer_parser.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":property_pointer",
        ":token_parser",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "property_pointer",
    srcs = [
        "property_pointer.cc",
    ],
    hdrs = [
        "property_pointer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":token_parser",
        "//core/math",
        "//core/ncsb",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "token_parser",
    srcs = [
        "token_parser.cc",
    ],
    hdrs = [
        "token_parser.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

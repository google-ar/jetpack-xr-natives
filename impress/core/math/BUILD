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
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//build_tools:imp.bzl", "imp_google3_copts")
load("//core/proto:imp_cc_proto_library.bzl", "imp_cc_proto_library")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "math",
    srcs = [
        "mat.cc",
        "math.cc",
        "quat.cc",
        "transform.cc",
    ],
    hdrs = [
        "almost_equal.h",
        "almost_equal_helper.h",
        "mat.h",
        "math.h",
        "quat.h",
        "transform.h",
        "vec.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//core/common:log",
        "//core/common:string_helpers",
        "//core/common:type_helpers",
        "@com_google_absl//absl/strings:str_format",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:math_io",
    ],
    alwayslink = 1,
)

cc_library(
    name = "aabb_helpers",
    hdrs = [
        "aabb_helpers.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":math",
        "//core/geometry/shapes",
    ],
    alwayslink = 1,
)

cc_library(
    name = "flatbuffer_support",
    srcs = [
        "flatbuffer_support.cc",
    ],
    hdrs = [
        "flatbuffer_support.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":math",
        "//core/common:fbs",
        "@com_github_google_flatbuffers//:flatbuffers",
    ],
    alwayslink = 1,
)

proto_library(
    name = "math_proto",
    srcs = ["math.proto"],
    deps = ["//core/proto:imp_proto"],
)

java_lite_proto_library(
    name = "math_java_proto_lite",
    deps = [":math_proto"],
)

cc_library(
    name = "mat_proto_message",
    hdrs = [
        "mat_proto_message.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":math",
        "//core/common:hash",
        "//core/proto:proto_common",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_proto_library(
    name = "math_g3_cc_proto",
    deps = [":math_proto"],
)

imp_cc_proto_library(
    name = "math_proto_cc",
    proto = ":math_proto",
    deps = [
        ":mat_proto_message",
        ":math",
        "@third_party//filament/filament:filament_hdrs",
    ],
)

proto_library(
    name = "arrays_proto",
    srcs = ["arrays.proto"],
    deps = [":math_proto"],
)

imp_cc_proto_library(
    name = "arrays_proto_cc",
    proto = ":arrays_proto",
    deps = [":math_proto_cc"],
)

cc_library(
    name = "moving_average",
    hdrs = [
        "moving_average.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    alwayslink = 1,
)

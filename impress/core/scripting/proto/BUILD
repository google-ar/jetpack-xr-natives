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
load(
    "//core/proto:imp_cc_proto_library.bzl",
    "imp_cc_proto_library",
)

# Protos to define messages from Javascript to C++ in Imp Web

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

proto_library(
    name = "bridge_proto",
    srcs = ["bridge.proto"],
    deps = [
        "@com_google_protobuf//:any_proto",
    ],
)

imp_cc_proto_library(
    name = "bridge_cc_proto",
    proto = ":bridge_proto",
    deps = [
        "//core/proto:any_imp_proto",
    ],
)

cc_library(
    name = "bridge_cc",
    deps = [":bridge_cc_proto"],
    alwayslink = 1,
)

java_lite_proto_library(
    name = "bridge_java_proto_lite",
    deps = [":bridge_proto"],
)

proto_library(
    name = "events_proto",
    srcs = ["events.proto"],
    deps = [
        "//core/ncsb:node_handle_message_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

imp_cc_proto_library(
    name = "events_cc_proto",
    proto = ":events_proto",
    deps = [
        "//core/ncsb:node_handle_message_imp_proto",
        "//core/proto:any_imp_proto",
    ],
)

cc_library(
    name = "events_cc",
    deps = [":events_cc_proto"],
    alwayslink = 1,
)

java_lite_proto_library(
    name = "events_java_proto_lite",
    deps = [":events_proto"],
)

proto_library(
    name = "api_proto",
    srcs = ["api.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/ncsb:node_handle_message_proto",
        "//core/proto:imp_proto",
        "//core/view/framework:framework_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

imp_cc_proto_library(
    name = "api_cc_proto",
    proto = ":api_proto",
    deps = [
        "//core/ncsb:node_handle_message_imp_proto",
        "//core/proto:any_imp_proto",
        "//core/view/framework:framework_proto_cc",
    ],
)

cc_library(
    name = "api_cc",
    deps = [":api_cc_proto"],
    alwayslink = 1,
)

java_lite_proto_library(
    name = "api_java_proto_lite",
    deps = [":api_proto"],
)

proto_library(
    name = "test_proto",
    srcs = ["test.proto"],
    deps = [
        "//core/proto:imp_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

imp_cc_proto_library(
    name = "test_cc_proto",
    proto = ":test_proto",
    deps = [
        "//core/proto:any_imp_proto",
    ],
)

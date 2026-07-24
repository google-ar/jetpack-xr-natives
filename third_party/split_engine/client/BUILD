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

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
    deps = ["@com_google_absl//absl/functional:any_invocable"],
    alwayslink = 1,
)

cc_library(
    name = "batch",
    srcs = ["batch.cc"],
    hdrs = ["batch.h"],
    deps = [
        ":types",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/functional:any_invocable",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_size_calculator",
        "@third_party//split_engine:flatbuffer_size_calculator",
        "@third_party//split_engine:split_engine_schema_version",
    ],
    alwayslink = 1,
)

cc_library(
    name = "batch_manager",
    srcs = ["batch_manager.cc"],
    hdrs = ["batch_manager.h"],
    deps = [
        ":batch",
        ":types",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/functional:any_invocable",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "transport",
    hdrs = ["transport.h"],
    deps = [
        ":types",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/functional:any_invocable",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "split_engine_ipc_client",
    srcs = ["split_engine_ipc_client.cc"],
    hdrs = ["split_engine_ipc_client.h"],
    deps = [
        ":batch",
        ":batch_manager",
        ":transport",
        ":types",
        "@com_google_absl//absl/functional:any_invocable",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_size_calculator",
    ],
    alwayslink = 1,
)

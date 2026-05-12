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

cc_library(
    name = "request_handler",
    srcs = ["request_handler.cc"],
    hdrs = ["request_handler.h"],
    deps = [
        "//core/async:future",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_renderer_context",
        "//core/split_engine:split_engine_shared_context",
        "@com_google_absl//absl/status",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "request_handler_registry",
    srcs = ["request_handler_registry.cc"],
    hdrs = ["request_handler_registry.h"],
    deps = [
        ":request_handler",
        "//core/common:small_source_location",
        "@com_google_absl//absl/base:no_destructor",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "built_in_material_request_handler",
    srcs = ["built_in_material_request_handler.cc"],
    hdrs = ["built_in_material_request_handler.h"],
    deps = [
        ":request_handler",
        ":request_handler_registry",
        "//core/async:future",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_renderer_context",
        "//core/split_engine/materials:split_engine_builtin_material_factory",
        "//core/split_engine/materials/builtin:builtin_material",
        "//core/view/framework:generic_materials",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@flatbuffers",
        "@third_party//split_engine:fbs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "add_custom_material_request_handler",
    srcs = ["add_custom_material_request_handler.cc"],
    hdrs = ["add_custom_material_request_handler.h"],
    deps = [
        ":request_handler",
        ":request_handler_registry",
        "//core:config",
        "//core/assets/material:material_load_options_proto_cc",
        "//core/async:future",
        "//core/common:trace",
        "//core/materials/compiler:runtime_material_compiler",
        "//core/materials/compiler:runtime_material_compiler_creator",
        "//core/split_engine:flatbuffer_utils",
        "//core/split_engine:split_engine_filament_resource_ptrs",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_renderer_context",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@flatbuffers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine:fbs",
        "@third_party//split_engine:fbs_deps",  # buildcleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "request_handlers",
    hdrs = ["request_handlers.h"],
    deps = [
        ":add_custom_material_request_handler",
        ":built_in_material_request_handler",
        ":request_handler",
        ":request_handler_registry",
    ],
    alwayslink = 1,
)

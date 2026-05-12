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
load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "imp_copts")
load(
    "//core/proto:imp_cc_proto_library.bzl",
    "imp_cc_proto_library",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

proto_library(
    name = "particle_emitter_state_proto",
    srcs = ["particle_emitter_state.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "particle_emitter_state_proto_cc",
    proto = ":particle_emitter_state_proto",
    deps = [
        "//core/math:math_proto_cc",
    ],
)

imp_assets(
    name = "test_data",
    srcs = [
        "test_data/cube.gltf",
    ],
    namespace = "imp_test_data",
)

cc_library(
    name = "particle",
    srcs = [
        "data_layout.cc",
        "node_particle_controller.cc",
        "particle_behavior.cc",
        "particle_data_provider.cc",
        "particle_emitter.cc",
        "particle_instance.cc",
        "particle_service.cc",
        "standard_particle_data_provider.cc",
    ],
    hdrs = [
        "data_layout.h",
        "node_particle_controller.h",
        "particle_behavior.h",
        "particle_controller.h",
        "particle_data_provider.h",
        "particle_emitter.h",
        "particle_emitter_info.h",
        "particle_instance.h",
        "particle_service.h",
        "standard_particle_data_provider.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":particle_emitter_state_proto_cc",
        "//core/assets",
        "//core/async:future",
        "//core/common:owned_ptr",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

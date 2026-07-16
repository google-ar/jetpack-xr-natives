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

# Target for the emitter state proto itself.
proto_library(
    name = "particle_emitter_state_proto",
    srcs = ["particle_emitter_state.proto"],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

# Wrapper for the emitter state proto into a C++ header.
imp_cc_proto_library(
    name = "particle_emitter_state_proto_cc",
    proto = ":particle_emitter_state_proto",
    deps = [
        "//core/math:math_proto_cc",
    ],
)

# Particle Instance, provides an interface to manipulate a single particle.
cc_library(
    name = "particle_instance",
    srcs = ["particle_instance.cc"],
    hdrs = ["particle_instance.h"],
    deps = [
        "//core/math",
        "//core/ncsb",
        "//core/particle/utils:data_provider",
        "//core/particle/utils:particle_format",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

# Particle Behavior, used to manage particle behaviors.
cc_library(
    name = "particle_behavior",
    hdrs = [
        "custom_particle_behavior.h",
        "particle_behavior_result.h",
        "particle_emitter_info.h",
    ],
    deps = [
        ":particle_instance",
        "//core/math",
    ],
    alwayslink = 1,
)

# Particle System, primary interface for creating and managing particle systems.
cc_library(
    name = "particle_system",
    srcs = ["particle_system.cc"],
    hdrs = ["particle_system.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":particle_behavior",
        ":particle_emitter_state_proto_cc",
        "//core/async:future",
        "//core/ncsb",
        "//core/particle/utils:particle_emitter",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
    ],
    alwayslink = 1,
)

# Test suite to run all tests.
test_suite(
    name = "all_particle_tests",
    tests = [
        ":particle_instance_test",
        ":particle_system_test",
    ],
)

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

licenses(["notice"])

# Particle Format, manages the format of particle data in memory.
cc_library(
    name = "particle_format",
    srcs = ["particle_format.cc"],
    hdrs = ["particle_format.h"],
    deps = [
        "//core/particle:particle_emitter_state_proto_cc",
    ],
    alwayslink = 1,
)

# Data Provider, manages the storage of particle data.
cc_library(
    name = "data_provider",
    srcs = [
        "data_provider.cc",
        "standard_data_provider.cc",
    ],
    hdrs = [
        "data_provider.h",
        "standard_data_provider.h",
    ],
    deps = [
        "//core/math",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

# Particle Pool, manages particle data for an emitter.
cc_library(
    name = "particle_pool",
    srcs = ["particle_pool.cc"],
    hdrs = ["particle_pool.h"],
    deps = [
        ":data_provider",
        ":particle_format",
        "//core/ncsb",
        "//core/particle:particle_emitter_state_proto_cc",
        "//core/particle:particle_instance",
    ],
    alwayslink = 1,
)

# Particle Behavior, manages the behavior of particles in the system.
cc_library(
    name = "particle_behavior_impl",
    srcs = [
        "particle_behavior.cc",
    ],
    hdrs = [
        "particle_behavior.h",
    ],
    deps = [
        "//core/math",
        "//core/ncsb",
        "//core/particle:particle_behavior",
        "//core/particle:particle_emitter_state_proto_cc",
        "//core/particle:particle_instance",
    ],
    alwayslink = 1,
)

# Particle Emitter, definition and implementation of different types of particle emitters.
cc_library(
    name = "particle_emitter",
    srcs = [
        "node_particle_emitter.cc",
        "particle_emitter.cc",
    ],
    hdrs = [
        "node_particle_emitter.h",
        "particle_emitter.h",
    ],
    deps = [
        ":particle_behavior_impl",
        ":particle_pool",
        "//core/assets",
        "//core/assets/gltf:gltf_asset",
        "//core/async:future",
        "//core/camera",
        "//core/common:owned_ptr",
        "//core/math",
        "//core/ncsb",
        "//core/particle:particle_behavior",
        "//core/particle:particle_emitter_state_proto_cc",
        "//core/particle:particle_instance",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

# Test suite to run all tests.
test_suite(
    name = "all_particle_impl_tests",
    tests = [
        ":data_provider_test",
        ":particle_behavior_impl_test",
        ":particle_emitter_test",
        ":particle_format_test",
        ":particle_pool_test",
    ],
)

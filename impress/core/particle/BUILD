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

# Data for particle-related tests.
imp_assets(
    name = "test_data",
    srcs = [
        "test_data/cube.gltf",
    ],
    namespace = "imp_test_data",
)

# Particle Format, manages the format of particle data in memory.
cc_library(
    name = "particle_format",
    srcs = ["particle_format.cc"],
    hdrs = ["particle_format.h"],
    deps = [
        ":particle_emitter_state_proto_cc",
    ],
    alwayslink = 1,
)

# Data Provider, manages the storage of particle data.
# TODO: (broken link) - Move internal libraries to utils/BUILD.
cc_library(
    name = "data_provider",
    srcs = [
        "utils/data_provider.cc",
        "utils/standard_data_provider.cc",
    ],
    hdrs = [
        "utils/data_provider.h",
        "utils/standard_data_provider.h",
    ],
    deps = [
        "//core/math",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

# Particle Instance, provides an interface to manipulate a single particle.
cc_library(
    name = "particle_instance",
    srcs = ["particle_instance.cc"],
    hdrs = ["particle_instance.h"],
    deps = [
        ":data_provider",
        ":particle_format",
        "//core/math",
        "//core/ncsb",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

# Particle Pool, manages particle data for an emitter.
cc_library(
    name = "particle_pool",
    srcs = ["utils/particle_pool.cc"],
    hdrs = ["utils/particle_pool.h"],
    deps = [
        ":data_provider",
        ":particle_emitter_state_proto_cc",
        ":particle_format",
        ":particle_instance",
        "//core/ncsb",
    ],
    alwayslink = 1,
)

# Particle Behavior, manages the behavior of particles in the system.
cc_library(
    name = "particle_behavior",
    srcs = ["utils/particle_behavior.cc"],
    hdrs = [
        "custom_particle_behavior.h",
        "particle_behavior_result.h",
        "particle_emitter_info.h",
        "utils/particle_behavior.h",
    ],
    deps = [
        ":particle_emitter_state_proto_cc",
        ":particle_instance",
        "//core/math",
        "//core/ncsb",
    ],
    alwayslink = 1,
)

# Particle Emitter, definition and implementation of different types of particle emitters.
cc_library(
    name = "particle_emitter",
    srcs = ["utils/node_particle_emitter.cc"],
    hdrs = [
        "utils/node_particle_emitter.h",
        "utils/particle_emitter.h",
    ],
    deps = [
        ":particle_behavior",
        ":particle_emitter_state_proto_cc",
        ":particle_instance",
        ":particle_pool",
        "//core/assets",
        "//core/async:future",
        "//core/common:owned_ptr",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
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
        ":particle_emitter",
        ":particle_emitter_state_proto_cc",
        "//core/async:future",
        "//core/ncsb",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
    ],
    alwayslink = 1,
)

# Test suite to run all particle-related tests.
test_suite(
    name = "all_particle_tests",
    tests = [
        ":data_provider_test",
        ":particle_behavior_test",
        ":particle_emitter_test",
        ":particle_format_test",
        ":particle_instance_test",
        ":particle_pool_test",
        ":particle_system_test",
    ],
)

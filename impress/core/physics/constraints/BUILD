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

# Impress Physics Constraints

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "if_dev_runtime")
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

cc_library(
    name = "constraints",
    srcs = [
        "base_constraint.cc",
        "cone_twist_constraint.cc",
        "generic_6dof_constraint.cc",
        "hinge_constraint.cc",
        "point2point_constraint.cc",
        "slider_constraint.cc",
    ],
    hdrs = [
        "base_constraint.h",
        "cone_twist_constraint.h",
        "generic_6dof_constraint.h",
        "hinge_constraint.h",
        "point2point_constraint.h",
        "slider_constraint.h",
    ],
    deps = [
        ":constraints_proto_cc",
        "//core:config",
        "//core/common:invocable",
        "//core/common:registry",
        "//core/math",
        "//core/ncsb",
        "//core/physics",
        "//core/physics:physics_helper",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
    ] + if_dev_runtime([
        "//core/common:debug_draw",
        "//core/physics:physics_constants",
        "//core/physics:physics_debug_draw",
    ]),
    alwayslink = 1,
)

proto_library(
    name = "constraints_proto",
    srcs = [
        "cone_twist_constraint_state.proto",
        "generic_6dof_constraint_state.proto",
        "hinge_constraint_state.proto",
        "point2point_constraint_state.proto",
        "slider_constraint_state.proto",
    ],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
        "//core/scene_handles:scene_handles_proto",
    ],
)

imp_cc_proto_library(
    name = "constraints_proto_cc",
    proto = ":constraints_proto",
)

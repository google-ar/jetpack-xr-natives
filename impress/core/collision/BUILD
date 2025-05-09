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

load("//build_tools:imp.bzl", "imp_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "ray",
    hdrs = ["ray.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = ["//core/math"],
    alwayslink = 1,
)

cc_library(
    name = "collision_flags",
    hdrs = ["collision_flags.h"],
    copts = imp_copts(),
    linkstatic = True,
    alwayslink = 1,
)

cc_library(
    name = "plane",
    hdrs = ["plane.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = ["//core/math"],
    alwayslink = 1,
)

cc_library(
    name = "bvh",
    srcs = ["bvh.cc"],
    hdrs = ["bvh.h"],
    deps = [
        ":ray",
        "//core/common:debug_draw",
        "//core/common:trace",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "bvh_collision_accelerator",
    srcs = ["bvh_collision_accelerator.cc"],
    hdrs = ["bvh_collision_accelerator.h"],
    deps = [
        ":bvh",
        ":collision_helpers",
        ":mesh_collision_accelerator",
        ":ray",
        "//core/common:debug_draw",
        "//core/geometry/shapes",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "collision_helpers",
    srcs = ["collision_helpers.cc"],
    hdrs = [
        "collision_helpers.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":plane",
        ":ray",
        "//core/geometry/shapes",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "collider_helpers",
    hdrs = [
        "collider_mask_helpers.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":collision_flags",
        "//core/common",
        "//core/common:crtp_helper",
    ],
    alwayslink = 1,
)

cc_library(
    name = "mesh_collision_accelerator",
    hdrs = [
        "mesh_collision_accelerator.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":collision_helpers",
        ":ray",
        "//core/geometry/shapes",
        "//core/model/mesh:mesh_data",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "collision_accelerator_provider",
    srcs = ["collision_accelerator_provider.cc"],
    hdrs = [
        "collision_accelerator_provider.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":bvh_collision_accelerator",
        ":mesh_collision_accelerator",
        "//core/geometry/shapes",
        "//core/model/mesh:mesh_data",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

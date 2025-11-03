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

# Impress Physics

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:assets.bzl", "imp_assets")
load(
    "//build_tools:imp.bzl",
    "if_dev_runtime",
    "imp_defines",
    "imp_google3_copts",
)
load("//build_tools:imp_material.bzl", "imp_material")
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
    name = "physics_helper",
    srcs = [
        "physics_helper.cc",
    ],
    hdrs = [
        "physics_helper.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/math",
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
    ],
    alwayslink = 1,
)

cc_library(
    name = "collidable_shape",
    srcs = [
        "collidable_shapes/collidable_shape.cc",
    ],
    hdrs = [
        "collidable_shapes/collidable_shape.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":physics_helper",
        "//core:config",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/types:optional",
    ],
    alwayslink = 1,
)

cc_library(
    name = "sphere_collidable_shape",
    srcs = [
        "collidable_shapes/sphere_collidable_shape.cc",
    ],
    hdrs = [
        "collidable_shapes/sphere_collidable_shape.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":collidable_shape",
        ":physics_helper",
        "//core:config",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
    ] + if_dev_runtime(["//core/common:debug_draw"]),
    alwayslink = 1,
)

cc_library(
    name = "box_collidable_shape",
    srcs = [
        "collidable_shapes/box_collidable_shape.cc",
    ],
    hdrs = [
        "collidable_shapes/box_collidable_shape.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":collidable_shape",
        ":physics_helper",
        "//core:config",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
    ] + if_dev_runtime(["//core/common:debug_draw"]),
    alwayslink = 1,
)

cc_library(
    name = "capsule_collidable_shape",
    srcs = ["collidable_shapes/capsule_collidable_shape.cc"],
    hdrs = ["collidable_shapes/capsule_collidable_shape.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":collidable_shape",
        ":physics_helper",
        "//core:config",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/log:check",
    ] + if_dev_runtime(["//core/common:debug_draw"]),
    alwayslink = 1,
)

cc_library(
    name = "cylinder_collidable_shape",
    srcs = ["collidable_shapes/cylinder_collidable_shape.cc"],
    hdrs = ["collidable_shapes/cylinder_collidable_shape.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":collidable_shape",
        ":physics_helper",
        "//core:config",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/log:check",
    ] + if_dev_runtime(["//core/common:debug_draw"]),
    alwayslink = 1,
)

cc_library(
    name = "cone_collidable_shape",
    srcs = ["collidable_shapes/cone_collidable_shape.cc"],
    hdrs = ["collidable_shapes/cone_collidable_shape.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":collidable_shape",
        ":physics_helper",
        "//core:config",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/log:check",
    ] + if_dev_runtime(["//core/common:debug_draw"]),
    alwayslink = 1,
)

cc_library(
    name = "compound_collidable_shape",
    srcs = [
        "collidable_shapes/compound_collidable_shape.cc",
    ],
    hdrs = [
        "collidable_shapes/collidable_shape_properties.h",
        "collidable_shapes/compound_collidable_shape.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":box_collidable_shape",
        ":capsule_collidable_shape",
        ":collidable_shape",
        ":cone_collidable_shape",
        ":cylinder_collidable_shape",
        ":physics_helper",
        ":physics_proto_cc",
        ":sphere_collidable_shape",
        "//core:config",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/log:check",
        "@com_google_impress//core/common:log",
    ] + if_dev_runtime(["//core/common:debug_draw"]),
    alwayslink = 1,
)

cc_library(
    name = "convex_hull_mesh_collidable_shape",
    srcs = [
        "collidable_shapes/convex_hull_mesh_collidable_shape.cc",
    ],
    hdrs = [
        "collidable_shapes/convex_hull_mesh_collidable_shape.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":collidable_shape",
        ":physics_helper",
        "//core:config",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/geometry/shapes",
        "//core/math",
        "//core/math:aabb_helpers",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
    ] + if_dev_runtime(["//core/common:debug_draw"]),
    alwayslink = 1,
)

cc_library(
    name = "static_mesh_collidable_shape",
    srcs = [
        "collidable_shapes/static_mesh_collidable_shape.cc",
    ],
    hdrs = [
        "collidable_shapes/static_mesh_collidable_shape.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":collidable_shape",
        ":physics_helper",
        "//core:config",
        "//core/common:filament_helpers",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/geometry/shapes",
        "//core/math",
        "//core/math:aabb_helpers",
        "//core/model/mesh",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
    ] + if_dev_runtime(["//core/common:debug_draw"]),
    alwayslink = 1,
)

cc_library(
    name = "physics",
    srcs = [
        "collidable.cc",
        "physics_manager.cc",
        "rigid_body.cc",
        "trigger_volume.cc",
    ],
    hdrs = [
        "collidable.h",
        "physics_manager.h",
        "rigid_body.h",
        "trigger_volume.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":box_collidable_shape",
        ":capsule_collidable_shape",
        ":collidable_shape",
        ":compound_collidable_shape",
        ":cone_collidable_shape",
        ":convex_hull_mesh_collidable_shape",
        ":cylinder_collidable_shape",
        ":physics_helper",
        ":physics_proto_cc",
        ":sphere_collidable_shape",
        ":static_mesh_collidable_shape",
        "//core:config",
        "//core/collision:ray",
        "//core/common:owned_ptr",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/scene_handles:scene_handles_imp_cc_proto",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:frame_time",
        "@bullet//:BulletCollision",  # build_cleaner: keep
        "@bullet//:BulletDynamics",  # build_cleaner: keep
        "@bullet//:LinearMath",
        "@com_google_absl//absl/debugging:stacktrace",
        "@com_google_absl//absl/debugging:symbolize",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ] + if_dev_runtime(["//core/common:debug_draw"]),
    alwayslink = 1,
)

proto_library(
    name = "physics_proto",
    srcs = [
        "collidable_shapes/collidable_shape_properties_state.proto",
        "collidable_type.proto",
        "rigid_body_state.proto",
    ],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "physics_proto_cc",
    proto = ":physics_proto",
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

imp_assets(
    name = "rigid_body_scuba_test_assets",
    srcs = [
        ":solid_color",
    ],
    namespace = "imp::rigidbody",
)

imp_material(
    name = "solid_color",
    src = ":solid_color.mat",
)

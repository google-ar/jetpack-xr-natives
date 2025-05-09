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

load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "imp_defines", "imp_google3_copts")
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

imp_assets(
    name = "assets",
    srcs = [
        ":line_3d",
    ],
    namespace = "imp::line_renderer",
)

imp_material(
    name = "line_3d",
    src = ":line_3d.mat",
    includes = [
        "line.glsl",
    ],
)

proto_library(
    name = "line_renderer_proto",
    srcs = [
        "line_renderer.proto",
    ],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "line_renderer_proto_cc",
    proto = ":line_renderer_proto",
    deps = [
        "//core/math:math_proto_cc",
    ],
)

cc_library(
    name = "line_renderer",
    srcs = [
        "line_renderer.cc",
    ],
    hdrs = [
        "line_renderer.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":assets",
        ":line_renderer_lib",
        ":line_renderer_proto_cc",
        "//:api",
        "//core/materials",
        "//core/math",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "line_renderer_lib",
    srcs = [
        "line_extruder_3d.cc",
        "stroke_polyline_util.cc",
    ],
    hdrs = [
        "line_buffer.h",
        "line_extruder_3d.h",
        "line_mat_types.h",
        "point_util.h",
        "polyline.h",
        "stroke_polyline_util.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":line_renderer_proto_cc",
        "//:api",
        "//core/common:log",
        "//core/geometry/shapes",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "line_renderer_test_utils",
    testonly = True,
    srcs = ["line_vertex_testing_utils.cc"],
    hdrs = [
        "line_vertex_testing_utils.h",
        "vector_testing_utils.h",
    ],
    deps = [
        ":line_renderer_lib",
        "//core/math",
        "@com_google_absl//absl/status",
        "@mediapipe//mediapipe/framework/port:gtest",
    ],
    alwayslink = 1,
)

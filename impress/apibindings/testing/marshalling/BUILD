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

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load(
    "//build_tools:imp.bzl",
    "if_non_android",
    "imp_google3_copts",
)

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "test_create_view_factory",
    testonly = True,
    srcs = ["test_create_view.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":test_impress_api_view",
        "//:api",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "test_model_manager",
    testonly = True,
    srcs = ["test_model_manager.cc"],
    hdrs = ["test_model_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":jni_test_utils",
        ":model_test_context",
        "//apibindings:base_asset_animator",
        "//apibindings:base_asset_loader",
        "//apibindings:impress_api_view_header",
        "//apibindings:model_manager_interface",
        "//core/geometry/shapes",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:gtest",
    ],
    alwayslink = 1,
)

cc_library(
    name = "test_skybox_manager",
    testonly = True,
    srcs = ["test_skybox_manager.cc"],
    hdrs = ["test_skybox_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":jni_test_utils",
        ":skybox_test_context",
        "//apibindings:base_asset_loader",
        "//apibindings:impress_api_view_header",
        "//apibindings:skybox_manager_interface",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:gtest",
    ],
    alwayslink = 1,
)

cc_library(
    name = "test_stereo_surface_manager",
    testonly = True,
    srcs = ["test_stereo_surface_manager.cc"],
    hdrs = ["test_stereo_surface_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//apibindings:impress_api_view_header",
        "//apibindings:stereo_surface",
        "//apibindings:stereo_surface_manager_interface",
        "//core/math",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/render:content_security_level",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "test_texture_manager",
    testonly = True,
    srcs = ["test_texture_manager.cc"],
    hdrs = ["test_texture_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":texture_test_context",
        "//apibindings:base_asset_loader",
        "//apibindings:impress_api_view_header",
        "//apibindings:texture_manager_interface",
        "//core/render",
        "//core/view/utils:asset",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "test_generic_material_manager",
    testonly = True,
    srcs = ["test_generic_material_manager.cc"],
    hdrs = ["test_generic_material_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//apibindings:base_asset_loader",
        "//apibindings:generic_material_manager_interface",
        "//apibindings:impress_api_view_header",
        "//core/material_library:generic_material_spec",
        "//core/math",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "test_water_material_manager",
    testonly = True,
    srcs = ["test_water_material_manager.cc"],
    hdrs = ["test_water_material_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//apibindings:base_asset_loader",
        "//apibindings:impress_api_view_header",
        "//apibindings:water_material_manager_interface",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "test_node_manager",
    testonly = True,
    srcs = ["test_node_manager.cc"],
    hdrs = ["test_node_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//apibindings:impress_api_view_header",
        "//apibindings:node_manager_interface",
        "//core/math",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "impress_api_test_helper",
    testonly = True,
    srcs = ["impress_api_test_helper.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":model_test_context",
        ":skybox_test_context",
        ":test_impress_api_view",
        ":texture_test_context",
        "//core/common:buffer_access",
        "//core/common:jni_helpers",
        "//core/input",
        "//core/material_library:fbs",
        "//core/material_library:generic_material_spec",
        "//core/math",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/ncsb",
        "//core/render:content_security_level",
        "//core/view/platforms/android/jni_helpers:exception_helper",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/status",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "test_impress_api_view",
    testonly = True,
    srcs = ["test_impress_api_view.cc"],
    hdrs = ["test_impress_api_view.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":test_generic_material_manager",
        ":test_model_manager",
        ":test_node_manager",
        ":test_skybox_manager",
        ":test_stereo_surface_manager",
        ":test_texture_manager",
        ":test_water_material_manager",
        "//apibindings:asset_ptr_map",
        "//apibindings:impress_api_view_header",
    ],
    alwayslink = 1,
)

cc_library(
    name = "model_test_context",
    testonly = True,
    hdrs = [
        "model_test_context.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    alwayslink = 1,
)

cc_library(
    name = "skybox_test_context",
    testonly = True,
    hdrs = [
        "skybox_test_context.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    alwayslink = 1,
)

cc_library(
    name = "texture_test_context",
    testonly = True,
    hdrs = [
        "texture_test_context.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    alwayslink = 1,
)

cc_library(
    name = "jni_test_utils",
    testonly = True,
    srcs = ["jni_test_utils.cc"],
    hdrs = ["jni_test_utils.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

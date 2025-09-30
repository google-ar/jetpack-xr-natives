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

# Library for providing cross-platform AR tracking.
load("//build_tools:imp.bzl", "if_android", "if_desktop", "if_dev_runtime", "if_emscripten", "if_ios", "imp_defines", "imp_google3_copts")
load("//core/proto:imp_cc_proto_library.bzl", "imp_cc_proto_library")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "ar",
    srcs = [
        "ar_session.cc",
    ],
    hdrs = [
        "ar_session.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    linkstatic = 1,
    deps = [
        ":ar_session_config",
        ":shared",
        "//core:config",
        "//core/ar/playback",
        "//core/ar/playback:playback_data",
        "//core/async:executor",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:filament_helpers",
        "//core/common:trace",
        "//core/common:tuple_helpers",
        "//core/math",
        "//core/view:base_view",
        "//core/window:window_rotation",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@robin_map",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ] + if_android([
        "//core/ar/android",
    ]) + if_dev_runtime([
        "//core/collision:collision_helpers",
        "//core/common:debug_draw",
        "//core/view/framework",
        "//:api",
    ]) + if_ios([
        "//core/ar/ios",
    ]) + if_desktop([
        "//core/ar/desktop",
    ]) + if_emscripten([
        "//core/ar/desktop",
    ]) + select({
        "//core/ar/android:impress_experimental": [
            "//third_party/arcore:c_api_private/experimental",
        ],
        "//core:android": [
            "//third_party/arcore:c_api_private/release",
        ],
        "//conditions:default": [],
    }),
    alwayslink = 1,
)

cc_library(
    name = "ar_session_config",
    hdrs = ["ar_session_config.h"],
    deps = ["@com_google_absl//absl/types:optional"],
    alwayslink = 1,
)

cc_library(
    name = "shared",
    srcs = [
        "ar_face.cc",
        "ar_plane.cc",
        "ar_session_native.cc",
        "ar_trackables_manager.cc",
    ],
    hdrs = [
        "ar_anchor.h",
        "ar_face.h",
        "ar_frame.h",
        "ar_hit_result.h",
        "ar_magical_surface_point.h",
        "ar_plane.h",
        "ar_point.h",
        "ar_point_cloud.h",
        "ar_prior_map.h",
        "ar_session_native.h",
        "ar_trackable.h",
        "ar_trackable_handle.h",
        "ar_trackable_helpers.h",
        "ar_trackable_id.h",
        "ar_trackables_manager.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":ar_face_imp_cc_proto",
        ":ar_hdr_lighting",
        ":ar_session_config",
        "//core:config",
        "//core/ar/playback:playback_data",
        "//core/async:future",
        "//core/collision:ray",
        "//core/common",
        "//core/common:robin_map",
        "//core/common:tuple_helpers",
        "//core/math",
        "//core/render",
        "//core/window:window_rotation",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:math_hdrs",
    ] + select({
        "//core/ar/android:impress_experimental": [
            "//third_party/arcore:c_api_private/experimental",
        ],
        "//core:android": [
            "//third_party/arcore:c_api_private/release",
            "//third_party/arcore/ar/core:feature_macros",
        ],
        "//conditions:default": [],
    }),
    alwayslink = 1,
)

cc_library(
    name = "mocks",
    testonly = True,
    hdrs = [
        "mock_ar_session.h",
        "mock_ar_session_native.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":ar",
        ":shared",
        "@com_google_absl//absl/status:statusor",
        "@mediapipe//mediapipe/framework/port:gtest_main",
    ],
    alwayslink = 1,
)

cc_library(
    name = "ar_hdr_lighting",
    hdrs = ["ar_hdr_lighting.h"],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        "//core/lighting:environment_light",
        "//core/math",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

proto_library(
    name = "ar_face_proto",
    srcs = ["ar_face.proto"],
)

imp_cc_proto_library(
    name = "ar_face_imp_cc_proto",
    proto = ":ar_face_proto",
)

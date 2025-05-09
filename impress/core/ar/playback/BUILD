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

load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//build_tools:imp.bzl", "imp_google3_copts")

# Library for providing tracking info for a video.

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "playback_data",
    hdrs = [
        "ar_playback_data.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":playback_scene_cc_proto",
        "//core/async:future",
        "//core/ncsb",
        "//core/video:video_controller",
    ],
    alwayslink = 1,
)

cc_library(
    name = "playback",
    srcs = [
        "ar_session_native_playback.cc",
    ],
    hdrs = [
        "ar_session_native_playback.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":playback_data",
        "//:api",
        "//core/ar:ar_session_config",
        "//core/ar:shared",
        "//core/collision:collision_helpers",
        "//core/common:log",
        "//core/video:video_controller",
        "//core/view:base_view",
        "//third_party/arcore/proto/video:ar_derived_data_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

proto_library(
    name = "playback_scene_proto",
    srcs = ["playback_scene.proto"],
    deps = [
        "//third_party/arcore/proto/video:ar_derived_data_proto",
    ],
)

cc_proto_library(
    name = "playback_scene_cc_proto",
    deps = [":playback_scene_proto"],
)

java_lite_proto_library(
    name = "playback_scene_java_proto_lite",
    deps = [":playback_scene_proto"],
)

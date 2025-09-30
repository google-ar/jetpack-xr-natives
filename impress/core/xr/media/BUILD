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
load("//build_tools:imp.bzl", "imp_google3_copts")
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

config_setting(
    name = "arm_v8",
    values = {"android_cpu": "arm64-v8a"},
)

proto_library(
    name = "xr_media_viewer_state_proto",
    srcs = ["xr_media_viewer_state.proto"],
)

imp_cc_proto_library(
    name = "xr_media_viewer_state_proto_cc",
    proto = ":xr_media_viewer_state_proto",
    deps = [
        "//core/proto:any_imp_proto",
    ],
)

cc_library(
    name = "utils",
    hdrs = [
        "utils.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/xr/media/data:assets",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "xr_media_viewer",
    srcs = [
        "xr_media_viewer.cc",
    ],
    hdrs = [
        "xr_media_viewer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":utils",
        ":xr_media_viewer_state_proto_cc",
        "//core:config",
        "//core/assets",
        "//core/async:future",
        "//core/common:robin_map",
        "//core/materials",
        "//core/math",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/ncsb",
        "//core/render",
        "//core/render/android:definitions",
        "//core/video:video_controller",
        "//core/video:video_controller_proto_cc",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "//core/xr/media/data:assets",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

# Requires --config=android_arm --android_ndk_min_sdk_version=34

filegroup(
    name = "xr_media_viewer_scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

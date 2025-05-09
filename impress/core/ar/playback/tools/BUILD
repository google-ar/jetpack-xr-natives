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

load("@bazel_skylib//:bzl_library.bzl", "bzl_library")
load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_binary(
    name = "ar_playback_scene_exporter",
    srcs = ["ar_playback_scene_exporter.cc"],
    copts = imp_google3_copts(),
    deps = [
        "//base:logging_extensions",
        "//core/ar/playback:playback_scene_cc_proto",
        "//file/base",
        "//file/localfile",
        "//third_party/arcore/proto/video:ar_derived_data_cc_proto",
        "@com_google_absl//absl/flags:flag",
        "@mediapipe//mediapipe/framework/port:status",
    ],
)

imp_assets(
    name = "scene_export",
    srcs = [
        "//core/ar/playback/test_data/playback_scene_source_data:scene_export",
    ],
    namespace = "scene_export",
)

imp_assets(
    name = "scene_export_with_buffers",
    srcs = [
        "//core/ar/playback/test_data/playback_scene_source_data:scene_export_with_buffers",
    ],
    namespace = "scene_export",
)

bzl_library(
    name = "imp_ar_playback_scene_data_bzl",
    srcs = ["imp_ar_playback_scene_data.bzl"],
    deps = [
        "//third_party/arcore/ar/sceneform/converter:converter_bzl",
    ],
)

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

# Helper build tools for Impress that are android-specific.

load("@bazel_skylib//:bzl_library.bzl", "bzl_library")

package(default_visibility = ["//visibility:public"])

exports_files(["proguard.pgcfg"])

bzl_library(
    name = "imp_android_app_bzl",
    srcs = ["imp_android_app.bzl"],
    deps = [
        ":imp_generate_android_binary_bzl",
        "//build_tools/platforms:imp_app_common_bzl",
    ],
)

bzl_library(
    name = "imp_app_resources_bzl",
    srcs = ["imp_app_resources.bzl"],
)

bzl_library(
    name = "imp_generate_android_binary_bzl",
    srcs = ["imp_generate_android_binary.bzl"],
    deps = [
        "//build_tools/platforms:imp_app_common_bzl",
        "//core/view/platforms/android:generate_activity_bzl",
        "@bazel_tools//tools/build_defs/android:bzl",
        "@bazel_tools//tools/build_defs/build_test",
    ],
)

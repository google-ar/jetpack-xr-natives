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

load("@build_bazel_rules_android//android:rules.bzl", "android_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

android_library(
    name = "media_no_jni",
    srcs = [
        "ImpMediaDataSource.java",
        "OnCompletionListener.java",
        "OnErrorListener.java",
        "OnInfoListener.java",
        "OnPreparedListener.java",
        "OnSeekCompleteListener.java",
    ],
    custom_package = "com.google.ar.imp.core.media",
    manifest = ":AndroidManifest.xml",
    resource_files = glob(["res/**"]),
    deps = ["@third_party//filament/android:filament_android_no_jni"],
)

android_library(
    name = "exoplayer_no_jni",
    srcs = [
        "ImpExoPlayer.java",
        "ImpExoPlayerListener.java",
    ],
    custom_package = "com.google.ar.imp.core.media",
    manifest = ":AndroidManifest.xml",
    deps = [
        "//third_party/java/android_libs/media:exoplayer",
        "//third_party/java/android_libs/media:exoplayer_dash",
        "//third_party/java/android_libs/media:exoplayer_hls",
        "@maven//:androidx_annotation_annotation",
        "@third_party//filament/android:filament_android_no_jni",
    ],
)

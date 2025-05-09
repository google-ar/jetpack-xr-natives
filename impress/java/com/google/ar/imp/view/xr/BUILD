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
    name = "imp_xr_api",
    srcs = ["ImpXrApi.java"],
    custom_package = "com.google.ar.imp.view.xr",
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
    ],
    manifest = ":AndroidManifest.xml",
    deps = [
        "//java/com/google/ar/imp/view",
        "//java/com/google/ar/imp/view:setup_params_java_proto_lite",
        "@maven//:androidx_annotation_annotation",
        "@maven//:androidx_appcompat_appcompat",
        "@maven//:com_google_guava_guava",
    ],
)

android_library(
    name = "imp_xr_renderer",
    srcs = ["ImpXrRenderer.java"],
    custom_package = "com.google.ar.imp.view.xr",
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
    ],
    manifest = ":AndroidManifest.xml",
    deps = [
        ":imp_xr_api",
        "//java/com/google/ar/imp/view",
        "//java/com/google/ar/imp/view:frame_scheduler",
        "//java/com/google/ar/imp/view:imp_api",
        "//java/com/google/ar/imp/view:setup_params_java_proto_lite",
        "@maven//:androidx_annotation_annotation",
        "@maven//:androidx_appcompat_appcompat",
        "@maven//:com_google_guava_guava",
    ],
)

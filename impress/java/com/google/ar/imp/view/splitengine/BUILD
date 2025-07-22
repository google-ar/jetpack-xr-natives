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
    name = "splitengine",
    srcs = [
        "ImpSplitEngine.java",
        "ImpSplitEngineApi.java",
        "ImpSplitEngineRenderer.java",
        "RendererConnectionServiceProvider.java",
    ],
    custom_package = "com.google.ar.imp.view.splitengine",
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
    ],
    tags = ["included_in_archive=//java/com/google/ar/imp/apibindings:impress_no_native_lib_aar"],
    deps = [
        "//core/split_engine/android/extensions/java/com/google/imp/splitengine/extensions:extensions_java",  # unuseddeps: keep
        "//java/com/google/ar/imp/view",
        "//java/com/google/ar/imp/view:frame_scheduler",
        "//java/com/google/ar/imp/view:imp_api_scuba",
        "@com_google_impress//java/com/google/vr/androidxr/extensions:platform_extensions_jar",
        "@maven//:androidx_annotation_annotation",
        "@maven//:androidx_concurrent_concurrent_futures",
        "@maven//:com_google_guava_guava",
    ],
)

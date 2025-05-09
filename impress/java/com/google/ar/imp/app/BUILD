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

exports_files([
    "AndroidManifest.xml",
    "arcore/AndroidManifest.xml",
    "testing/AndroidManifest_empty.xml",
    "xr/AndroidManifest.xml",
    "ic_launcher.png",
    "styles.xml",
])

android_library(
    name = "basic_imp_activity",
    srcs = ["BasicImpActivity.java"],
    deps = [
        "//java/com/google/ar/imp/view:imp_api",
        "//java/com/google/ar/imp/view:setup_params_java_proto_lite",
        "//java/com/google/ar/imp/view:surface_view_no_jni",
        "@maven//:androidx_appcompat_appcompat",
    ],
)

android_library(
    name = "basic_imp_fragment_activity",
    srcs = ["BasicImpFragmentActivity.java"],
    deps = [
        "//java/com/google/ar/imp/core/web:fragment_host",
        "//java/com/google/ar/imp/view:imp_api",
        "//java/com/google/ar/imp/view:setup_params_java_proto_lite",
        "//java/com/google/ar/imp/view:surface_view_no_jni",
        "@maven//:androidx_fragment_fragment",
    ],
)

android_library(
    name = "basic_imp_xr_activity",
    srcs = ["BasicImpXrActivity.java"],
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
        "-Xep:PackageLocation:OFF",
    ],
    manifest = "xr/AndroidManifest.xml",
    deps = [
        "//java/com/google/ar/imp/view/xr:imp_xr_renderer",
        "@maven//:androidx_appcompat_appcompat",
    ],
)

android_library(
    name = "basic_imp_split_engine_activity",
    srcs = ["BasicImpSplitEngineActivity.java"],
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
        "-Xep:PackageLocation:OFF",
    ],
    manifest = "//java/com/google/ar/imp/app/splitengine:AndroidManifest.xml",
    deps = [
        "//java/com/google/ar/imp/view:imp_api_scuba_proxy_test_view_no_jni",
        "//java/com/google/ar/imp/view/splitengine",
        "@maven//:androidx_appcompat_appcompat",
        "@third_party//jetpack_xr/java/com/google/vr/realitycore/runtime/androidxr/extensions:platform_extensions_provider",
    ],
)

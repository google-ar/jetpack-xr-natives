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
load("//build_tools:imp.bzl", "if_android", "if_desktop", "if_imp_split_engine_allow_experimental_apis", "if_non_android")
load("//build_tools:imp_app.bzl", "imp_app")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

imp_app(
    name = "split_engine_renderer_view",
    srcs = ["split_engine_renderer_view.cc"],
    android_activity_name = "SplitEngineRendererActivity",
    android_base_activity = "//java/com/google/ar/imp/app/splitengine:split_engine_renderer_activity",
    android_manifest = "//java/com/google/ar/imp/app/splitengine:SplitEngineRendererAndroidManifest.xml",
    android_package = "com.google.ar.imp.app.splitengine",
    multidex = "native",
    platforms = [
        "android",
        "desktop",
    ],
    tags = ["nobuilder"],
    deps = [
        "//core/common:debug_draw",
        "//core/input",
        "//core/math",
        "//core/ncsb",
        "//core/split_engine:split_engine_renderer",
        "//core/split_engine:split_engine_renderer_impl",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:variant",
    ] + if_android([":split_engine_jni"]) + if_desktop([
        "//core/view/platforms/desktop_split_engine:desktop_split_engine_renderer_hook",
    ]) + if_imp_split_engine_allow_experimental_apis([
        "@third_party//split_engine:split_engine_schema_version",
    ]),
)

cc_library(
    name = "split_engine_jni",
    srcs = ["split_engine_jni.cc"],
    deps = [
        ":view_update_params",
        "//:platform_android",
        "//core/async:executor",
        "//core/common:buffer_access",
        "//core/common:enum_flags",
        "//core/common:jni_helpers",
        "//core/common:platform_helpers",
        "//core/common:registry",
        "//core/math",
        "//core/split_engine:split_engine_bridge_sender",
        "//core/split_engine:split_engine_serializer_impl",
        "//core/split_engine/android:split_engine_android_bridge",
        "//core/split_engine/android:split_engine_android_shared_memory_bridge",
        "//core/split_engine/android:split_engine_shared_memory_bridge_client",
        "//core/split_engine/android:split_engine_shared_memory_bridge_sender",
        "//core/split_engine/android/extensions:split_engine_bridge",
        "//core/split_engine/android/extensions:split_engine_bridge_jni",
        "//core/view:base_view",
        "//core/view:view_host",
        "//core/view/framework",
        "//core/view/platforms/xr_android:openxr_includes",
        "//core/view/platforms/xr_android:xr_helpers",
        "//core/window:filament_host",
        "//core/xr:openxr_loader_lib",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@third_party//filament",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "view_update_params",
    srcs = ["view_update_params_jni.cc"],
    hdrs = ["view_update_params.h"],
    deps = [
        "//core/common:jni_helpers",
        "//core/math",
    ] + if_non_android([
        "@bazel_tools//tools/jdk:jni",
    ]),
    alwayslink = 1,
)

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
load("//build_tools:imp.bzl", "if_dev_runtime")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

android_library(
    name = "input_event_handler",
    srcs = ["InputEventHandler.java"],
    custom_package = "com.google.ar.imp.view",
    manifest = ":AndroidManifest.xml",
    proguard_specs = [
        ":proguard-rules.pro",
    ],
)

android_library(
    name = "input_event_source",
    srcs = ["InputEventSource.java"],
    custom_package = "com.google.ar.imp.view",
    manifest = ":AndroidManifest.xml",
    proguard_specs = [
        ":proguard-rules.pro",
    ],
    deps = [
        ":input_event_handler",
    ],
)

android_library(
    name = "texture_view_no_jni",
    srcs = ["ImpTextureView.java"],
    custom_package = "com.google.ar.imp.view",
    manifest = ":AndroidManifest.xml",
    proguard_specs = [
        ":proguard-rules.pro",
    ],
    deps = [
        ":imp_api",
        ":imp_api_scuba",
        ":input_event_handler",
        ":input_event_source",
        ":setup_params_java_proto_lite",
        "//java/com/google/ar/imp/core/web:fragment_host",
        "@maven//:androidx_annotation_annotation",
    ],
)

android_library(
    name = "surface_view_no_jni",
    srcs = ["ImpSurfaceView.java"],
    custom_package = "com.google.ar.imp.view",
    manifest = ":AndroidManifest.xml",
    proguard_specs = [
        ":proguard-rules.pro",
    ],
    deps = [
        ":imp_api",
        ":imp_api_scuba",
        ":input_event_handler",
        ":input_event_source",
        ":setup_params_java_proto_lite",
        "//java/com/google/ar/imp/core/web:fragment_host",
        "@maven//:androidx_annotation_annotation",
        "@maven//:com_google_guava_guava",
    ],
)

android_library(
    name = "imp_api_scuba_proxy_test_view_no_jni",
    srcs = ["ImpApiScubaProxyTestView.java"],
    custom_package = "com.google.ar.imp.view",
    manifest = ":AndroidManifest.xml",
    proguard_specs = [
        ":proguard-rules.pro",
    ],
    deps = [
        ":imp_api_scuba",
        "//java/com/google/ar/imp/core/web:fragment_host",
        "@maven//:androidx_annotation_annotation",
    ],
)

android_library(
    name = "frame_scheduler",
    srcs = [
        "ChoreographerFrameScheduler.java",
        "ContinuousFrameScheduler.java",
        "FrameScheduler.java",
    ],
    manifest = ":AndroidManifest.xml",
    proguard_specs = [
        ":proguard-rules.pro",
    ],
    deps = [
        ":choreographer_holder",
        "@maven//:androidx_annotation_annotation",
        "@maven//:androidx_concurrent_concurrent_futures",
        "@maven//:com_google_guava_guava",
    ],
)

android_library(
    name = "view",
    srcs = [
        "View.java",
    ],
    manifest = ":AndroidManifest.xml",
    proguard_specs = [
        ":proguard-rules.pro",
    ],
    tags = [
        "included_in_archive=//java/com/google/ar/imp/apibindings:impress_no_native_lib_aar",
    ],
    deps = [
        "//java/com/google/ar/imp/core/scripting:script_endpoint",
        "//java/com/google/ar/imp/core/web:fragment_host",
        "//java/com/google/ar/imp/view/input:input_manager",
        "@maven//:androidx_annotation_annotation",
    ] + if_dev_runtime([
        "//java/com/google/ar/imp/core/input:input",
    ]),
)

android_library(
    name = "imp_api_scuba",
    srcs = [
        "ImpApiScuba.java",
        "ImpApiScubaProvider.java",
    ],
    manifest = ":AndroidManifest.xml",
    proguard_specs = [
        ":proguard-rules.pro",
    ],
    deps = [
        "@maven//:androidx_annotation_annotation",
    ],
)

android_library(
    name = "imp_api",
    srcs = [
        "ImpApi.java",
        "ImpApiProvider.java",
        "ImpViewController.java",
    ],
    manifest = ":AndroidManifest.xml",
    proguard_specs = [
        ":proguard-rules.pro",
    ],
    deps = [
        ":frame_scheduler",
        ":imp_api_scuba",
        ":setup_params_java_proto_lite",
        ":view",
        "//java/com/google/ar/imp/core/media:media_no_jni",  # buildcleaner: keep
        "//java/com/google/ar/imp/core/scripting:script_endpoint",
        "//java/com/google/ar/imp/core/web:fragment_host",
        "//java/com/google/ar/imp/view/input:input_manager",
        "@maven//:androidx_annotation_annotation",
        "@maven//:com_google_guava_guava",
        "@third_party//filament/android:filament_android_no_jni",
        "@third_party//filament/android:ui_helper",
    ] + if_dev_runtime([
        "//java/com/google/ar/imp/core/input:input",
    ]),
)

android_library(
    name = "choreographer_holder",
    srcs = [
        "AndroidChoreographerHolder.java",
        "ChoreographerHolder.java",
    ],
    proguard_specs = [
        ":proguard-rules.pro",
    ],
)

proto_library(
    name = "setup_params_proto",
    srcs = ["setup_params.proto"],
)

java_lite_proto_library(
    name = "setup_params_java_proto_lite",
    deps = [":setup_params_proto"],
)

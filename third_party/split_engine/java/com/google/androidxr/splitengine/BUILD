# Copyright 2025 Google LLC
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

android_library(
    name = "split_engine_subspace_manager",
    srcs = [
        "SplitEngineSubspaceManager.java",
    ],
    custom_package = "com.google.androidxr.splitengine",
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
    ],
    tags = ["included_in_archive=//java/com/google/ar/imp/apibindings:impress_no_native_lib_aar"],
    deps = [
        ":scene_leash_node_factory",
        ":split_engine_input_event",
        ":subspace_node",
        "@com_google_impress//java/com/google/ar/imp/view",
        "@com_google_impress//java/com/google/ar/imp/view:frame_scheduler",
        "@com_google_impress//java/com/google/ar/imp/view/splitengine",
        "@maven//:androidx_annotation_annotation",
        "@third_party//jetpack_xr/java/com/google/vr/realitycore/runtime/androidxr/extensions:platform_extensions_jar",
        "@third_party//jetpack_xr/java/com/google/vr/realitycore/runtime/androidxr/extensions:xrextensionslib",
    ],
)

android_library(
    name = "subspace_node",
    srcs = [
        "SubspaceNode.java",
    ],
    custom_package = "com.google.androidxr.splitengine",
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
    ],
    tags = ["included_in_archive=//java/com/google/ar/imp/apibindings:impress_no_native_lib_aar"],
    deps = [
        "@third_party//jetpack_xr/java/com/google/vr/realitycore/runtime/androidxr/extensions:platform_extensions_jar",
    ],
)

android_library(
    name = "split_engine_input_event",
    srcs = [
        "SplitEngineInputEvent.java",
    ],
    custom_package = "com.google.androidxr.splitengine",
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
    ],
    deps = [
        "@maven//:androidx_annotation_annotation",
        "@third_party//jetpack_xr/java/com/google/vr/realitycore/runtime/androidxr/extensions:platform_extensions_jar",
    ],
)

android_library(
    name = "split_engine_view_update_provider",
    srcs = [
        "SplitEngineViewUpdateProvider.java",
    ],
    custom_package = "com.google.androidxr.splitengine",
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
    ],
    deps = [
        "//java/com/google/vr/realitycore/runtime/androidxr:jxr_platform_adapter_axr",
        "//java/com/google/vr/realitycore/runtime/androidxr/perception:fov",
        "//java/com/google/vr/realitycore/runtime/androidxr/perception:pose",
        "//java/com/google/vr/realitycore/runtime/androidxr/perception:view_projection",
        "//java/com/google/vr/realitycore/runtime/androidxr/perception:view_projections",
        "@com_google_impress//java/com/google/ar/imp/view/splitengine",
        "@maven//:androidx_annotation_annotation",
        "@maven//:com_google_guava_guava",
        "@third_party//jetpack_xr/java/com/google/vr/realitycore/runtime/androidxr/extensions:platform_extensions_jar",
    ],
)

android_library(
    name = "scene_leash_node_factory",
    srcs = [
        "SceneLeashNodeFactory.java",
    ],
    custom_package = "com.google.androidxr.splitengine",
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
    ],
    deps = [
        "@third_party//jetpack_xr/java/com/google/vr/realitycore/runtime/androidxr/extensions:platform_extensions_jar",
    ],
)

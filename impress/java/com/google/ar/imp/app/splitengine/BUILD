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
    "SplitEngineRendererAndroidManifest.xml",
])

PROGUARD_IPC = [
    "ipc_proguard.flags",
]

android_library(
    name = "split_engine_shared_memory_bridge_service_java",
    srcs = [
        "SplitEngineSharedMemoryBridgeService.java",
    ],
    proguard_specs = PROGUARD_IPC,
)

android_library(
    name = "split_engine_renderer_activity",
    srcs = ["SplitEngineRendererActivity.java"],
    manifest = "//java/com/google/ar/imp/app/splitengine:SplitEngineRendererAndroidManifest.xml",
    deps = [
        "//java/com/google/ar/imp/app:basic_imp_activity",
        "//java/com/google/ar/imp/app/splitengine:split_engine_shared_memory_bridge_service_java",  # buildcleaner: keep # unuseddeps: keep
        "//java/com/google/ar/imp/view:surface_view_no_jni",
    ],
)

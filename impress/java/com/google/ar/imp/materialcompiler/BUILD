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

licenses(["notice"])

android_library(
    name = "material_compiler_aidl",
    idl_import_root = "aidl",
    idl_srcs = [
        "aidl/IMaterialCompilerService.aidl",
    ],
)

android_library(
    name = "material_compiler_no_jni",
    srcs = [
        "MaterialCompilerClient.java",
        "MaterialCompilerService.java",
    ],
    exports_manifest = 1,
    javacopts = [
        "-Xep:AndroidJdkLibsChecker:OFF",
    ],
    manifest = "AndroidManifest.xml",
    deps = [
        ":material_compiler_aidl",
        "@maven//:androidx_annotation_annotation",
    ],
)

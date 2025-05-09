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

load("@build_bazel_rules_android//android:rules.bzl", "aar_import")
load("@rules_java//java:defs.bzl", "java_import")

package(default_visibility = ["//visibility:public"])

aar_import(
    name = "xrextensionslib",
    aar = "androidx.xr.extensions-release.aar",
    package = "androidx.xr.extensions",
    deps = [
        ":platform_extensions_jar",
        "@maven//:androidx_annotation_annotation",
        "@maven//:androidx_annotation_annotation_experimental",
        "@maven//:com_google_guava_guava",
    ],
)

# The API for the XR extensions.
#
# This jar is just the *stubs* for the API, making it analogous to the header
# file of a dynamically loaded library.
#
# It should NOT be used for the actual implementation of the API. To enforce
# this, its neverlink attribute is enabled.
java_import(
    name = "platform_extensions_jar",
    constraints = ["android"],
    jars = ["com.android.extensions.xr.jar"],
    neverlink = 1,
)

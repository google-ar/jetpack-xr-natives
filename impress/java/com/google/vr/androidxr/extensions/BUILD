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

load("@rules_java//java:defs.bzl", "java_import")

package(default_visibility = ["//visibility:public"])

#######################################################################
# The following jars are built from PMAD.  They are imported into g3:
#  - For Impress when building the Impress AAR
#  - For JXR during an AOSP drop into g3 (needed to support the AARs)
#  - For JXR 1P apps that directly use the JARs or include them via JXR
#
# PMAD check in: (broken link)
# Android build id: (broken link)
#######################################################################

# The API for the XrExtensions.
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

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

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "perception_library_jni",
    srcs = ["perception_library_jni.cc"],
    local_defines = [
        "XR_USE_PLATFORM_ANDROID",
        "XR_EXTENSION_PROTOTYPES",
        "XR_USE_TIMESPEC",
    ],
    deps = [
        ":jni_utils",
        "//third_party/OpenXR_KHR:openxr_headers_androidxr",
        "@abseil-cpp//absl/base",
        "@abseil-cpp//absl/log",
        "//openxr:openxr_manager",
    ],
    alwayslink = True,
)

cc_library(
    name = "jni_utils",
    srcs = ["jni_utils.cc"],
    hdrs = ["jni_utils.h"],
    deps = [
        "//third_party/OpenXR_KHR:openxr_headers_androidxr",  # buildcleaner: keep
        "@abseil-cpp//absl/strings:str_format",
        "//common:jni_utils",
    ],
    alwayslink = True,
)

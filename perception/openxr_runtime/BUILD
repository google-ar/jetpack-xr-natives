# Copyright 2026 Google LLC
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

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

JNI_SRCS = glob(["*_jni.cc"])

JNI_DEPS = [
    ":openxr_instance_manager",
    ":openxr_instance_manager_utils",
    "@third_party//OpenXR_KHR:generated_openxr_headers_public",
    "@third_party//OpenXR_KHR:openxr_headers_androidxr",
    "//common:jni_utils",
    "@abseil-cpp//absl/strings:str_format",
]

DEFINES = [
    "XR_USE_PLATFORM_ANDROID",
    "XR_EXTENSION_PROTOTYPES",
    "XR_USE_TIMESPEC",
]

cc_library(
    name = "lib",
    srcs = JNI_SRCS,
    defines = DEFINES,
    deps = JNI_DEPS,
    alwayslink = 1,
)

# Add this target to the deps of your app's android_binary rule to use the OpenXR module.
cc_binary(
    name = "libandroidx.xr.runtime.openxr.so",
    srcs = JNI_SRCS,
    linkopts = [
        "-landroid",
        "-lbinder_ndk",
    ],
    linkshared = 1,
    deps = JNI_DEPS + [
        "@third_party//OpenXR_KHR:openxr_loader",
    ],
)

cc_binary(
    name = "libandroidx.xr.runtime.openxr.test.so",
    testonly = 1,
    srcs = JNI_SRCS,
    linkopts = [
        "-landroid",
        "-lbinder_ndk",
    ],
    linkshared = 1,
    deps = JNI_DEPS + [
        "//common:stub",
    ],
)

cc_library(
    name = "openxr_instance_manager",
    srcs = ["openxr_instance_manager.cc"],
    hdrs = ["openxr_instance_manager.h"],
    defines = DEFINES,
    deps = [
        "@abseil-cpp//absl/base:core_headers",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/synchronization",
        "@abseil-cpp//absl/types:span",
        "//common:openxr_utils",
        "@third_party//OpenXR_KHR:generated_openxr_headers_public",  # build_cleaner: keep
        "@third_party//OpenXR_KHR:openxr_headers_androidxr",  # build_cleaner: keep
    ],
    alwayslink = 1,
)

cc_library(
    name = "openxr_instance_manager_utils",
    srcs = [
        "jobject_creator.cc",
    ],
    hdrs = [
        "jobject_creator.h",
    ],
    defines = DEFINES,
    deps = [
        "@abseil-cpp//absl/strings:str_format",
        "//common:namespace_util",
        "@third_party//OpenXR_KHR:generated_openxr_headers_public",  # build_cleaner: keep
        "@third_party//OpenXR_KHR:openxr_headers_androidxr",  # build_cleaner: keep
    ],
    alwayslink = 1,
)

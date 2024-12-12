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

JNI_SRCS = glob(["*_jni.cc"])

JNI_DEPS = [
    ":openxr_manager",
    ":openxr_manager_utils",
    "@abseil-cpp//absl/log",
    "//third_party/OpenXR_KHR:openxr_headers_androidxr",
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
)

# This is required to enable SceneViewerXR to be able to use the OpenXR instance/session.
# SceneViewerXR's usage of this should go away after XRU.
#
# Using OpenXR from multiple .so libraries is (apparently) broken, likely because it breaks the
# One Definition Rule, possibly inside libstdc++.
#
# See: https://developer.android.com/ndk/guides/cpp-support#ic
config_setting(
    name = "openxr_mergable_jni_libraries",
    flag_values = {
        "//third_party/android/ndk:jni_libraries_merge_spec": "//third_party/android/ndk:merge_all",
    },
)

# Add this target to the deps of your app's android_binary rule to use the OpenXR module.
cc_binary(
    name = "libandroidx.xr.openxr.so",
    srcs = JNI_SRCS,
    linkopts = [
        "-landroid",
        "-lbinder_ndk",
    ],
    linkshared = 1,
    deps = JNI_DEPS + [
        "//third_party/OpenXR_KHR:openxr_loader",
        "//perception_library:perception_library_jni",
    ],
)

cc_library(
    name = "stub",
    testonly = 1,
    srcs = ["openxr_stub.cc"],
    defines = DEFINES,
    deps = JNI_DEPS + [
        "@abseil-cpp//absl/container:flat_hash_map",
        "@abseil-cpp//absl/strings:string_view",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libandroidx.xr.openxr.test.so",
    testonly = 1,
    srcs = JNI_SRCS,
    linkopts = [
        "-landroid",
        "-lbinder_ndk",
    ],
    linkshared = 1,
    deps = JNI_DEPS + [":stub"],
)

cc_library(
    name = "openxr_manager",
    srcs = ["openxr_manager.cc"],
    hdrs = ["openxr_manager.h"],
    defines = DEFINES,
    deps = [
        ":openxr_manager_clock",
        ":openxr_manager_utils",
        "//third_party/OpenXR_KHR:openxr_headers_androidxr",
        "@abseil-cpp//absl/base:core_headers",
        "@abseil-cpp//absl/base:no_destructor",
        "@abseil-cpp//absl/container:flat_hash_map",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/numeric:int128",
        "@abseil-cpp//absl/synchronization",
    ],
    alwayslink = True,
)

cc_library(
    name = "openxr_manager_utils",
    srcs = [
        "jobject_converter.cc",
        "jobject_creator.cc",
        "openxr_manager_utils.cc",
    ],
    hdrs = [
        "jobject_converter.h",
        "jobject_creator.h",
        "openxr_manager_utils.h",
    ],
    defines = DEFINES,
    deps = [
        "//third_party/OpenXR_KHR:openxr_headers_androidxr",
        "@abseil-cpp//absl/numeric:int128",
        "@abseil-cpp//absl/strings:str_format",
        "//common:jni_utils",
    ],
)

cc_library(
    name = "openxr_manager_clock",
    srcs = [
        "openxr_manager_clock.cc",
    ],
    hdrs = ["openxr_manager_clock.h"],
    deps = [
        "@abseil-cpp//absl/synchronization",
        "@abseil-cpp//absl/time",
    ],
)

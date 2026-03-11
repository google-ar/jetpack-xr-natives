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

load("@bazel_skylib//:bzl_library.bzl", "bzl_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:objc_library.bzl", "objc_library")
load("//build_tools:imp.bzl", "if_android", "if_ios", "imp_default_jni_linkopts")
load("//core/scripting/testing:build_defs.bzl", "scripting_end_to_end_test", "standalone_jasmine_test_script")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "android_test_runner",
    testonly = 1,
    srcs = [
        "android/web_view_jni.cc",
        "android/web_view_test_runner.cc",
    ],
    hdrs = [
        "web_view_test_runner.h",
    ],
    linkopts = imp_default_jni_linkopts(),
    deps = [
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//testing",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

objc_library(
    name = "ios_test_runner",
    testonly = 1,
    srcs = [
        "ios/IMPTestWebView.mm",
        "ios/web_view_test_runner.mm",
    ],
    hdrs = [
        "ios/IMPTestWebView.h",
        "web_view_test_runner.h",
    ],
    sdk_dylibs = [
        "libc++",
    ],
    deps = [
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:context",
        "//testing",
        "//third_party/apple_frameworks:Foundation",
        "//third_party/apple_frameworks:UIKit",
        "//third_party/apple_frameworks:WebKit",
        "//third_party/apple_frameworks:XCTest",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "scripting_test_view_default_jni",
    testonly = 1,
    srcs = if_android(["android/scripting_test_jni.cc"]),
    deps = [
        ":scripting_test",
        "//core/common:context",
        "//core/common:jni_helpers",
    ],
    alwayslink = 1,
)

cc_library(
    name = "scripting_test",
    testonly = 1,
    srcs = [
        "web_view_test_runner.cc",
    ],
    hdrs = [
        "scripting_test.h",
        "web_view_test_runner.h",
    ],
    deps = [
        "//core/async:future",
        "//core/common:buffer_access",
        "//core/common:context",
        "//testing",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ] + if_android([
        ":android_test_runner",
        "//core/common:jni_helpers",
    ]) + if_ios([
        ":ios_test_runner",
    ]),
    alwayslink = 1,
)

standalone_jasmine_test_script(
    name = "scripting_end_to_end_test_script",
    #????
    srcs = ["//javascript/testing:scripting_end_to_end_test.js"],
    namespace = "imp::scripting",
)

cc_library(
    name = "scripting_end_to_end_test_script_cc",
    deps = ["scripting_end_to_end_test_script"],
    alwayslink = 1,
)

# Build commands to successfully run this test on various platforms:
#
# Android:
# blaze test //core/scripting/testing:scripting_android_endtoend_test \
# --android_ndk_min_sdk_version=24 \
# --grte_top=//third_party/android/ndk/platforms/android-24:everything --config=android_x86
#
# iOS:
# blaze test //core/scripting/testing:scripting_ios_endtoend_test
#
# WASM:
# blaze test //core/scripting/testing:scripting_wasm_endtoend_test
scripting_end_to_end_test(
    name = "scripting",
    srcs = [
        "scripting_end_to_end_test.cc",
    ],
    android_test_deps = [
        ":scripting_test_view_default_jni",
        "//core/view/framework/tests/data:test_model_resources",
    ],
    cc_deps = [
        ":scripting_end_to_end_test_script_js",
        ":scripting_test",
        "//core/async:future",
        "//core/common",
        "//core/common:buffer_access",
        "//core/resources",
        "//core/scripting",
        "//core/scripting:basic_api",
        "//core/scripting:scripting_system_web",
        "//core/scripting/proto:test_cc_proto",
        "//core/scripting/web:web_view",
        "//testing",
        "//testing:base_executor_test_helper",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
    ],
    ios_test_deps = [
        "//core/view/framework/tests/data:test_model_resources_lib",
    ],
    java_deps = [
        "//java/com/google/ar/imp/core/web:imp_web_view_no_jni",
        "//java/com/google/ar/imp/core/web/testing:web_view_for_test_no_jni",
    ],
    test_tags = [
        "notap",  #TODO Remove notap when fixed
    ],
)

bzl_library(
    name = "build_defs_bzl",
    srcs = ["build_defs.bzl"],
    deps = [
        "//build_tools:assets_bzl",
        "//build_tools:imp_bzl",
        "//testing:build_defs_bzl",
        "//testing/web/build_defs:js",
        "//third_party/arcore/ar/sceneform/dev:portable_embed_data_bzl",
    ],
    # TODO: Restore visibility restriction.
    #visibility = ["//visibility:private"],
)

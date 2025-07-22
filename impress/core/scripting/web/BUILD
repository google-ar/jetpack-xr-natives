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

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:objc_library.bzl", "objc_library")
load("//build_tools:imp.bzl", "if_native_debug_enabled", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "web_view",
    srcs = [
        "web_view.cc",
    ],
    hdrs = [
        "web_view.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:buffer_access",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/math",
        "//core/ncsb",
        "//core/proto:any_imp_proto",
        "//core/scripting:message_handler",
        "//core/scripting/proto:bridge_cc_proto",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/scripting",
        "//core/view/utils:asset",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "web_view_desktop",
    srcs = [
        "desktop/web_view.cc",
    ],
    hdrs = [
        "desktop/web_view.h",
    ],
    deps = [
        ":web_view",
        "//core/view:base_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "web_view_android",
    srcs = [
        "android/jni/scripting_bridge_jni.cc",
        "android/jni/web_view_bridge_jni.cc",
        "android/jni/web_view_fragment_jni.cc",
        "android/web_view.cc",
    ],
    copts = imp_google3_copts(),
    defines = if_native_debug_enabled(["IMP_WEB_ENABLE_CHROME_DEBUG=1"]),
    linkstatic = 1,
    deps = [
        ":web_view",
        "//core/common:jni_helpers",
        "//core/common:log",
        "//core/proto:proto_reader",
        "//core/proto:proto_writer",
        "//core/scripting:message_helpers",
        "//core/scripting/proto:bridge_cc_proto",
        "//core/view:view_host",
        "//core/view/scripting",
    ],
    alwayslink = 1,
)

objc_library(
    name = "web_view_ios",
    srcs = [
        "ios/IMPWebView.mm",
        "ios/web_view.mm",
    ],
    hdrs = [
        "ios/IMPWebView.h",
        "ios/web_view.h",
    ],
    features = ["-layering_check"],
    sdk_dylibs = [
        "libc++",
    ],
    deps = [
        ":web_view",
        "//core/common:context",
        "//core/common:log",
        "//core/scripting:message_helpers",
        "//core/scripting/proto:bridge_cc_proto",
        "//core/view/framework",
        "//third_party/apple_frameworks:Foundation",
        "//third_party/apple_frameworks:UIKit",
        "//third_party/apple_frameworks:WebKit",
    ],
)

cc_library(
    name = "wasm_helpers",
    hdrs = [
        "wasm/wasm_helpers.h",
    ],
    tags = [
        "manual",
        "nobuilder",
        "notap",
    ],
    alwayslink = 1,
)

cc_library(
    name = "web_view_wasm",
    srcs = [
        "wasm/web_view.cc",
    ],
    hdrs = [
        "wasm/web_view.h",
    ],
    tags = [
        "manual",
        "nobuilder",
        "notap",
    ],
    deps = [
        ":wasm_helpers",
        ":web_view",
        "//core/common",
        "//core/common:log",
        "//core/common:rememberer",
        "//core/scripting:message_helpers",
        "//core/view/framework",
        "//javascript/core:imp_web_bridge_js_embed",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
    ],
    alwayslink = 1,
)

cc_library(
    name = "web_key_codes",
    srcs = ["web_key_codes.cc"],
    hdrs = [
        "web_key_codes.h",
    ],
    deps = [
        "//core/input",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "mock_web_view",
    testonly = True,
    hdrs = [
        "mock_web_view.h",
    ],
    deps = [
        ":web_view",
        "//core/scripting/proto:bridge_cc_proto",
        "//testing",
        "@mediapipe//mediapipe/framework/port:gtest_main",
    ],
    alwayslink = 1,
)

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
load(
    "//build_tools:imp.bzl",
    "if_android",
    "if_dev_runtime",
    "if_ios",
    "if_non_android",
    "imp_copts",
    "imp_google3_copts",
)
load(
    "//core/proto:imp_cc_proto_library.bzl",
    "imp_cc_proto_library",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "input",
    srcs = [
        "input_manager.cc",
        "key_codes.cc",
        "keyboard_event.cc",
        "pointer_event.cc",
        "pointer_event_processor.cc",
        "wheel_event.cc",
    ],
    hdrs = [
        "input_manager.h",
        "key_codes.h",
        "keyboard_event.h",
        "pointer_event.h",
        "pointer_event_processor.h",
        "textinput_event.h",
        "wheel_event.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":pointer_event_cc_proto",
        "//core:config",
        "//core/actions:input_action_event",
        "//core/collision:collision_flags",
        "//core/common",
        "//core/common:trace",
        "//core/math",
        "//core/ncsb",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
        "@third_party//filament/libs:math_hdrs",
    ] + if_dev_runtime(["@dear_imgui"]),
    alwayslink = 1,
)

cc_library(
    name = "dev_mode_input_interceptor",
    srcs = [
        "dev_mode_input_interceptor.cc",
    ],
    hdrs = [
        "dev_mode_input_interceptor.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":input",
        ":keyboard_controller",
        "//core/actions:action_config",
        "//core/actions:controller_events",
        "//core/actions:input_action_event",
        "//core/common:registry",
        "//core/common:trace",
        "//core/editor",
        "//core/editor:editor_info",
        "//core/editor/widgets/viewport:viewport_widget",
        "//core/editor/xr:xr_editor_ui",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/window:filament_host",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

# Proto library for PointerEvent
proto_library(
    name = "pointer_event_proto",
    srcs = [
        "pointer_event.proto",
    ],
    deps = ["//core/math:math_proto"],
)

cc_library(
    name = "keyboard_controller",
    srcs = ["keyboard_controller.cc"],
    hdrs = [
        "keyboard_controller.h",
        "platform_keyboard_controller.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/common:context",
        "//core/view:base_view",
    ] + if_android([":android_keyboard"]) + if_ios([":ios_keyboard"]),
    alwayslink = 1,
)

cc_library(
    name = "android_keyboard",
    srcs = ["android/android_keyboard_controller.cc"],
    hdrs = [
        "android/android_keyboard_controller.h",
        "platform_keyboard_controller.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/view:base_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "clipboard_event",
    hdrs = ["clipboard_event.h"],
    deps = ["@com_google_absl//absl/strings:string_view"],
    alwayslink = 1,
)

objc_library(
    name = "ios_keyboard",
    srcs = [
        "ios/ios_keyboard_controller.mm",
    ],
    hdrs = [
        "ios/ios_keyboard_controller.h",
        "platform_keyboard_controller.h",
    ],
    deps = [
        ":input",
        "//core/async:executor",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view:view_events",
        "//third_party/apple_frameworks:CoreText",
        "//third_party/apple_frameworks:UIKit",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
)

# Javascript flavor of pointer_event_proto generated code

# Imp flavor CC proto library for the PointerEvent, a common message type.
imp_cc_proto_library(
    name = "pointer_event_cc_proto",
    proto = ":pointer_event_proto",
)

# Proto library for KeyboardEvent
proto_library(
    name = "keyboard_event_proto",
    srcs = [
        "keyboard_event.proto",
    ],
)

# Javascript flavor of keyboard_event_proto generated code

# Imp flavor CC proto library for the KeyboardEvent, a common message type.
imp_cc_proto_library(
    name = "keyboard_event_cc_proto",
    proto = ":keyboard_event_proto",
)

# Proto library for WheelEvent
proto_library(
    name = "wheel_event_proto",
    srcs = [
        "wheel_event.proto",
    ],
    deps = ["//core/math:math_proto"],
)

# Javascript flavor of wheel_event_proto generated code

# Imp flavor CC proto library for the WheelEvent.
imp_cc_proto_library(
    name = "wheel_event_cc_proto",
    proto = ":wheel_event_proto",
)

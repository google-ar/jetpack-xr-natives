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

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "desktop_split_engine_renderer_hook",
    srcs = [
        "desktop_renderer_hook.cc",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/async:executor",
        "//core/common:registry",
        "//core/split_engine/desktop:split_engine_desktop_bridge_service_impl",
        "//core/split_engine/desktop/multimachine:split_engine_desktop_bridge_service",
        "//core/split_engine/desktop/multimachine:split_engine_desktop_bridge_service_impl_local",
        "//core/view:base_view",
        "//core/view/platforms/desktop:desktop_hooks",
        "//third_party/grpc:grpc++",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

# An incomplete library which is combined via .bzl with a provider of the client API.
cc_library(
    name = "desktop_split_engine_serializer",
    srcs = [
        "desktop_serializer.cc",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/common:context",
        "//core/input",
        "//core/split_engine:split_engine_serializer_impl",
        "//core/split_engine/android:split_engine_android_bridge",
        "//core/split_engine/android:split_engine_android_shared_memory_bridge",
        "//core/split_engine/android:split_engine_shared_memory_bridge_sender",
        "//core/split_engine/desktop:split_engine_desktop_bridge_cc_grpc",
        "//core/split_engine/desktop:split_engine_desktop_bridge_client",
        "//core/split_engine/desktop/multimachine:split_engine_desktop_bridge_cc_grpc",
        "//core/split_engine/desktop/multimachine:split_engine_desktop_bridge_client",
        "//core/split_engine/desktop/multimachine:split_engine_desktop_bridge_client_impl",
        "//core/split_engine/desktop/multimachine:split_engine_desktop_bridge_legacy",
        "//core/split_engine/desktop/multimachine:split_engine_desktop_bridge_sender",
        "//core/view:view_host",
        "//core/view/framework",
        "//core/view/utils:device",
        "//core/window:filament_host",
        "//core/window:sdl_venue",
        "//third_party/grpc:grpc++",
        "@com_google_absl//absl/debugging:failure_signal_handler",
        "@com_google_absl//absl/debugging:symbolize",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

filegroup(
    name = "desktop_split_engine_samples",
    srcs = [
        "//core/split_engine/android/view:split_engine_renderer_view_desktop",
        "//prototypes/samples/split_engine_serializer_stress_test:split_engine_serializer_stress_test_desktop_split_engine",
        "//samples/morph_desktop_split_engine:morph_stress_test_desktop_split_engine",
        "//samples/simple:simple_desktop_split_engine",
    ],
    data = [
        "//core/split_engine/android/view:split_engine_renderer_view_desktop",
        "//prototypes/samples/split_engine_serializer_stress_test:split_engine_serializer_stress_test_desktop_split_engine",
        "//samples/morph_desktop_split_engine:morph_stress_test_desktop_split_engine",
        "//samples/simple:simple_desktop_split_engine",
    ],
)

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

load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "if_android", "if_desktop", "if_emscripten", "if_ios", "if_non_android", "imp_google3_copts")
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

# Main Imp audio library that compiles the appropriate platform-specific
# libraries depending on which platform it is intended for. Note that
# for Android, an additional dependency on
# //java/com/google/ar/imp/core/media:media_no_jni
# will be needed for the apk build itself to access the JNI functions.
# TODO Implement desktop audio to replace the noop one used for it.
cc_library(
    name = "audio",
    srcs = [
        "audio_player.cc",
    ],
    hdrs = [
        "audio_player.h",
        "audio_source.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":audio_proto_cc",
        "//core:config",
        "//core/assets",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:log",
        "//core/common:trace",
        "//core/media",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/async:future",
        "//core/view/framework",
        "//core/view/utils:asset",
        "@com_google_absl//absl/functional:bind_front",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework/port:status",
    ] + if_ios([
        ":ios_audio",
    ]) + if_android([
        ":android_audio",
    ]) + if_desktop([
        ":noop_audio",
    ]) + if_emscripten([
        ":wasm_audio",
    ]),
    alwayslink = 1,
)

# TODO: Refactor platform implementations to remove need to export same header here and below.
cc_library(
    name = "noop_audio",
    srcs = [
        "noop_audio_source.cc",
    ],
    hdrs = [
        "audio_source.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/assets",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:registry",
        "//core/media",
        "//core/view:base_view",
        "//core/view/framework",
        "//proposed:delayed_future_scheduler",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/time",
    ],
    alwayslink = 1,
)

cc_library(
    name = "android_audio",
    srcs = [
        "android_audio_source.cc",
    ],
    hdrs = [
        "audio_source.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/assets",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/common:trace",
        "//core/media",
        "//core/media:android_media",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework/port:status",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

objc_library(
    name = "ios_audio",
    srcs = [
        "ios_audio_source.mm",
    ],
    hdrs = [
        "audio_source.h",
    ],
    features = ["-layering_check"],
    deps = [
        "//core/async:future",
        "//core/media",
        "//core/view:base_view",
        "//core/view/framework",
        "//third_party/apple_frameworks:AVFoundation",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "wasm_audio",
    srcs = [
        "wasm_audio_source.cc",
    ],
    hdrs = [
        "audio_source.h",
    ],
    copts = imp_google3_copts(),
    linkopts = [
        "--post-js $(location //javascript/core/wasm/audio:audio.js.lds)",
    ],
    tags = ["nobuilder"],
    deps = [
        "//core/assets",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:log",
        "//core/common:platform_helpers",
        "//core/common:resource_helpers",
        "//core/media",
        "//core/resources",
        "//core/view:base_view",
        "//core/view/framework",
        "//javascript/core/wasm/audio:audio.js.lds",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

proto_library(
    name = "audio_proto",
    srcs = ["audio_player_state.proto"],
)

imp_cc_proto_library(
    name = "audio_proto_cc",
    proto = ":audio_proto",
    deps = [
        "//core/proto:any_imp_proto",
    ],
)

imp_assets(
    name = "test_resources",
    srcs = [
        ":test_data/valid_audio.mp3",
    ],
    namespace = "imp::audio::test_data",
)

# Creating a cc_library just to wrap test_resources, as the obj_library
# build command does not take non-cc_libraries as dependencies.
cc_library(
    name = "test_resources_cc",
    deps = [
        ":test_resources",
    ],
    alwayslink = 1,
)

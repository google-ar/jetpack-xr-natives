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
load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "if_android", "if_desktop", "if_dev_runtime", "if_emscripten", "if_ios", "if_linux", "if_macos", "imp_google3_copts")
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
    name = "video",
    srcs = [
        "video_player.cc",
    ] + if_dev_runtime(["video_player_widget_helper.cc"]),
    hdrs = [
        "video_player.h",
    ] + if_dev_runtime(["video_player_widget_helper.h"]),
    copts = imp_google3_copts(),
    deps = [
        ":video_proto_cc",
        ":video_source",
        "//core:config",
        "//core/assets",
        "//core/async:future",
        "//core/common:context",
        "//core/math",
        "//core/media",
        "//core/ncsb",
        "//core/render",
        "//core/video/data:video_assets",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:asset",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ] + if_android([
        ":android_video",
    ]) + if_linux([
        ":linux_video",
    ]) + if_macos([
        ":macos_video",
    ]) + if_emscripten([
        ":wasm_video",
    ]) + if_ios([
        ":ios_video",
    ]) + if_dev_runtime([
        "@dear_imgui",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "video_source",
    hdrs = [
        "video_source.h",
        "video_source_factory.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/async:future",
        "//core/common:context",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/render",
        "//core/render/android:definitions",
        "//core/view:base_view",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ] + if_android([
        "//core/media:android_media",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "android_video",
    srcs = [] + if_android([
        "android_video_source.cc",
    ]) + if_desktop([
        "noop_video_source.cc",
    ]),
    copts = imp_google3_copts(),
    deps = [
        ":video_source",
        "//core:config",
        "//core/assets",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media",
        "//core/media:android_media",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/render",
        "//core/render/android:android_external_texture_surface",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//gl",
    ],
    alwayslink = 1,
)

cc_library(
    name = "exoplayer_video_source_factory",
    srcs = ["exoplayer_video_source_factory.cc"],
    hdrs = ["exoplayer_video_source_factory.h"],
    copts = imp_google3_copts(),
    deps = [
        ":android_exoplayer_video",
        ":video_source",
        "//core/async:future",
        "//core/render/android:android_external_texture_surface",
        "//core/render/android:definitions",
        "//core/resources",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "android_exoplayer_video",
    srcs = ["android_exoplayer_video_source.cc"],
    hdrs = [
        "android_exoplayer_video_source.h",
        "video_source.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":video_source",
        "//core/async:future",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media",
        "//core/media:android_exoplayer_media",
        "//core/media:android_exoplayer_media_jni",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/render",
        "//core/render:content_security_level",
        "//core/render/android:android_external_texture_surface",
        "//core/render/android:definitions",
        "//core/resources",
        "//core/view:base_view",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

objc_library(
    name = "ios_video",
    srcs = [
        "ios_video_source.mm",
    ],
    deps = [
        ":video_source",
        "//core/common",
        "//core/media",
        "//core/media:media_color_space",
        "//core/render",
        "//third_party/apple_frameworks:AVFoundation",
        "//third_party/apple_frameworks:CoreMedia",
        "//third_party/apple_frameworks:Foundation",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
    ],
)

filegroup(
    name = "video_scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

cc_library(
    name = "linux_video",
    srcs = [
        "linux_video_source.cc",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":video_source",
        "//core/assets",
        "//core/async:executor",
        "//core/async:future",
        "//core/common",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/render",
        "//core/resources",
        "//core/view:base_view",
        "//core/view/framework",
        "//video/common:temp_filename_deleter",
        "//video/common/subprocess/ffmpeg",
        "//video/common/subprocess/ffmpeg:ffmpeg_cc_proto",
        "//video/file/shared/pipe:pipe_reader",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@re2",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)

objc_library(
    name = "macos_video",
    srcs = [
        "macos_video_source.mm",
    ],
    deps = [
        ":video_source",
        "//core/async:future",
        "//core/media",
        "//core/media:media_color_space",
        "//core/render",
        "//core/view:base_view",
        "//third_party/apple_frameworks:AVFoundation",
        "//third_party/apple_frameworks:CoreMedia",
        "//third_party/apple_frameworks:Foundation",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
)

cc_library(
    name = "noop_video",
    srcs = [
        "noop_video_source.cc",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":video_source",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/render",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "wasm_video",
    srcs = [
        "wasm_video_source.cc",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":video_source",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/render",
        "//core/view:base_view",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/time",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

proto_library(
    name = "video_proto",
    srcs = ["video_player_state.proto"],
    deps = [
        "//core/proto:imp_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

imp_cc_proto_library(
    name = "video_proto_cc",
    proto = ":video_proto",
    deps = [
        "//core/proto:any_imp_proto",
    ],
)

imp_assets(
    name = "test_assets",
    srcs = [
        "//core/video/test_data:portrait_video.mp4",
        "//core/video/test_data:video.mp4",
    ],
    namespace = "imp::video::test_data",
)

cc_library(
    name = "test_assets_cc",
    deps = [
        ":test_assets",
    ],
    alwayslink = 1,
)

filegroup(
    name = "video_player_scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

cc_library(
    name = "video_writer",
    hdrs = [
        "video_writer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/async:future",
        "//core/common",
        "//core/math",
        "//core/window:filament_host",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
    ] + if_ios(
        [":video_writer_ios"],
        otherwise = [":video_writer_noop"],
    ),
    alwayslink = 1,
)

cc_library(
    name = "video_recorder",
    srcs = [
        "video_recorder.cc",
    ],
    hdrs = [
        "video_recorder.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":video_writer",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/math",
        "//core/media",
        "//core/ncsb",
        "//core/render",
        "//core/video/data:video_assets",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/framework",
        "//core/view/utils:asset",
        "//core/window:filament_host",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@third_party//filament/filament:filament_hdrs",
    ] + if_ios(
        [":video_writer_ios"],
        otherwise = [":video_writer_noop"],
    ),
    alwayslink = 1,
)

cc_library(
    name = "video_writer_noop",
    srcs = ["video_writer_noop.cc"],
    hdrs = [
        "video_writer.h",
    ],
    deps = [
        "//core/async:future",
        "//core/common",
        "//core/math",
        "//core/window:filament_host",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

objc_library(
    name = "video_writer_ios",
    srcs = [
        "video_file_writer_ios.mm",
        "video_writer_ios.mm",
        "web_server_video_stream_writer_ios.mm",
    ],
    hdrs = [
        "video_writer.h",
        "video_writer_frame.h",
    ],
    copts = imp_google3_copts(),
    features = ["-layering_check"],
    deps = [
        "//core/async:future",
        "//core/common",
        "//core/math",
        "//core/ui_stream:web_server_ios",
        "//core/window:filament_host",
        "//third_party/apple_frameworks:AVFoundation",
        "//third_party/apple_frameworks:CoreMedia",
        "//third_party/apple_frameworks:CoreVideo",
        "//third_party/apple_frameworks:VideoToolbox",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:backend_hdrs",
    ],
)

proto_library(
    name = "video_controller_proto",
    srcs = ["video_controller_state.proto"],
)

imp_cc_proto_library(
    name = "video_controller_proto_cc",
    proto = ":video_controller_proto",
)

cc_library(
    name = "android_or_exoplayer_video",
    deps = select({
        "//core:imp_android_external_texture_surface_uses_image_reader": [
            ":android_exoplayer_video",
            ":exoplayer_video_source_factory",
        ],
        "//conditions:default": [":android_video"],
    }),
    alwayslink = 1,
)

cc_library(
    name = "video_controller",
    srcs = [
        "video_controller.cc",
    ] + if_dev_runtime(["video_controller_widget_helper.cc"]),
    hdrs = [
        "video_controller.h",
    ] + if_dev_runtime(["video_controller_widget_helper.h"]),
    copts = imp_google3_copts(),
    deps = [
        ":video_controller_proto_cc",
        ":video_source",
        "//core:config",
        "//core/async:future",
        "//core/common:context",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/ncsb",
        "//core/render",
        "//core/render/android:android_external_texture_surface",
        "//core/render/android:definitions",
        "//core/video/data:video_assets",
        "//core/view:base_view",
        "//core/view/utils:asset",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ] + select({
        "//core:linux": [":linux_video"],
        "//core:macos": [":macos_video"],
        "//core:ios": [":ios_video"],
        "@third_party//filament:wasm": [":wasm_video"],
        "//core:android": [":android_or_exoplayer_video"],
        "//conditions:default": [":noop_video"],
    }) + if_dev_runtime([
        "@dear_imgui",
    ]),
    alwayslink = 1,
)

proto_library(
    name = "basic_video_viewer_proto",
    srcs = ["basic_video_viewer_state.proto"],
)

imp_cc_proto_library(
    name = "basic_video_viewer_proto_cc",
    proto = ":basic_video_viewer_proto",
)

cc_library(
    name = "basic_video_viewer",
    srcs = [
        "basic_video_viewer.cc",
    ],
    hdrs = [
        "basic_video_viewer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":basic_video_viewer_proto_cc",
        ":video_controller",
        "//core/async:future",
        "//core/math",
        "//core/media",
        "//core/ncsb",
        "//core/video/data:video_assets",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

filegroup(
    name = "basic_video_viewer_scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

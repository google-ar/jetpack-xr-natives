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

# The impel/window folder wraps the bits universal to all platforms and all contexts
# for rendering a 3D image to a framebuffer.  Not to be confused with imp/core/view, the
# client-facing view interface for imp.
# impel/window:filament_host is particularly useful as a bare bones, completely
# unopinionated way to create and interact with a filament engine.  impel/viewer uses
# filament host and applies certain universal opinions, like a default IBL, and a set of
load("@com_google_sandboxed_api//sandboxed_api/bazel:embed_data.bzl", "sapi_cc_embed_data")
load("@third_party//filament:filament.bzl", "by_backend")
load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "if_android", "if_dev_runtime", "if_emscripten", "imp_copts", "imp_defines", "imp_google3_copts")
load("//build_tools:imp_image_based_lighting.bzl", "imp_image_based_lighting")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

#boop
licenses(["notice"])

cc_library(
    name = "window_rotation",
    hdrs = [
        "window_rotation.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    tags = ["nobuilder"],
    deps = [
        "//core/common:log",
        "//core/common:platform_helpers",
    ],
    alwayslink = 1,
)

filegroup(
    name = "dev_fonts",
    srcs = [
        "//core/fonts:googlesans_medium.ttf",
    ],
)

sapi_cc_embed_data(
    name = "imp_dev_resources",
    srcs = [
        ":dev_fonts",
    ],
    flatten = True,
)

cc_library(
    name = "dev_mode_extension",
    srcs = [
        "dev_mode_extension.cc",
    ],
    hdrs = [
        "dev_mode_extension.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":default_dev_mode_extension",
        ":filament_host",
    ],
    alwayslink = 1,
)

cc_library(
    name = "default_dev_mode_extension",
    srcs = [
        "default_dev_mode_extension.cc",
    ],
    hdrs = [
        "default_dev_mode_extension.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":filament_host",
        ":imp_dev_resources",
        "//core/common",
        "//core/common:debug_draw",
        "//core/common:log",
        "//core/common:typed_ids",
        "//core/input",
        "//core/math",
        "//core/window/clipboard:clipboard_handler",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@dear_imgui",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:filagui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "filament_host",
    srcs = [
        "filament_host.cc",
        "filament_view.cc",
        "shared_host_state.cc",
    ],
    hdrs = [
        "filament_host.h",
        "filament_host_input.h",
        "filament_view.h",
        "shared_host_state.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    linkstatic = True,
    tags = ["nobuilder"],
    deps = [
        ":window_rotation",
        "//core:config",
        "//core/async:executor_helpers",
        "//core/async:simple_executor",
        "//core/async:thread_pool_executor",
        "//core/common",
        "//core/common:filament_engine_helpers",
        "//core/common:log",
        "//core/common:pass_key",
        "//core/common:robin_map",
        "//core/common:trace",
        "//core/common:typed_ids",
        "//core/math",
        "//core/monitor",
        "//core/window/clipboard:clipboard_handler",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/types:variant",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
        "@third_party//filament/libs:utils_hdrs",
    ] + if_android([
        "@third_party//filament/filament/backend:backend_hdrs",
    ]) + by_backend(
        vulkan = [
            "@third_party//filament/filament/backend:backend_vk_hdrs",
        ],
    ) + if_dev_runtime([
        "@stblib//:stb_image_write",
    ]) + if_emscripten(
        [],
        [
        ],
    ),
    alwayslink = 1,
)

imp_image_based_lighting(
    name = "default_ibl",
    size = 8,
    src = "default_lighting_env.exr",
    blur_roughness = 0.05,
    ibl_samples = 4096,
)

imp_assets(
    name = "default_lighting",
    srcs =
        [
            ":default_ibl",
        ],
    namespace = "imp",
)

objc_library(
    name = "native_window_helper_cocoa",
    srcs = ["native_window_helper_cocoa.mm"],
    hdrs = ["native_window_helper.h"],
    tags = [
        "manual",
        "nobuilder",
    ],
    deps = [
        "//third_party/apple_frameworks:Cocoa",
        "@SDL2",
    ],
)

cc_library(
    name = "native_window_helper_linux",
    srcs = ["native_window_helper_linux.cc"],
    hdrs = ["native_window_helper.h"],
    copts = imp_copts(),
    linkstatic = 1,
    tags = [
        "manual",
        "nobuilder",
    ],
    deps = [
        "@SDL2",
    ],
    alwayslink = 1,
)

cc_library(
    name = "native_window_helper_windows",
    srcs = ["native_window_helper_windows.cc"],
    hdrs = ["native_window_helper.h"],
    copts = imp_copts(),
    linkstatic = 1,
    tags = [
        "manual",
        "nobuilder",
    ],
    deps = [
        "@SDL2",
    ],
    alwayslink = 1,
)

alias(
    name = "native_window_helper",
    actual = select({
        "@third_party//filament:windows": ":native_window_helper_windows",
        "@third_party//filament:macos": ":native_window_helper_cocoa",
        "@third_party//filament:wasm": ":native_window_helper_linux",
        "//conditions:default": ":native_window_helper_linux",
    }),
)

cc_library(
    name = "sdl_venue",
    srcs = ["sdl_venue.cc"],
    hdrs = ["sdl_venue.h"],
    deps = [
        ":filament_host",
        ":native_window_helper",
        ":sdl_input_processor",
        "//core:config",
        "//core/common",
        "//core/common:enum_flags",
        "//core/common:filament_engine_helpers",
        "//core/common:log",
        "//core/common:platform_helpers",
        "//core/input",
        "//core/view/platforms/desktop/clipboard:sdl_clipboard_handler",
        "//core/view/utils:device",
        "@SDL2",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "sdl_input_processor",
    srcs = ["sdl_input_processor.cc"],
    hdrs = ["sdl_input_processor.h"],
    deps = [
        ":filament_host",
        "//core:config",
        "//core/input",
        "@SDL2",
        "@com_google_absl//absl/status",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "golden_image_helpers",
    testonly = 1,
    srcs = ["golden_image_helpers.cc"],
    hdrs = ["golden_image_helpers.h"],
    copts = imp_copts(),
    linkstatic = 1,
    tags = ["nobuilder"],
    deps = [
        "//core/common",
        "//image/wimage",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

# For a more maintainable build this target should not exist and the headers
# should  be split into the existing cc_library targets, but this change was
# automatically  done so that we can remove long standing issues and complexity
# in the build system. It's up to the OWNERS of this package to get rid of it or
# not. The use of the textual_hdrs attribute is discouraged, use hdrs instead.
# Here it is used to avoid header parsing errors in packages where the feature
# parse_headers was enabled since loose headers were not being parsed. See
# (broken link) for more details.
cc_library(
    name = "loose_headers",
    tags = ["avoid_dep"],
    textual_hdrs = ["dev_mode_extension.h"],
    alwayslink = 1,
)

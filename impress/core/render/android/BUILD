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
load("//build_tools:imp.bzl", "if_android_external_texture_surface_uses_image_reader", "imp_defines", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "definitions",
    hdrs = ["android_defines.h"],
    alwayslink = 1,
)

cc_library(
    name = "android_external_texture_surface",
    srcs = ["android_external_texture_surface.cc"],
    hdrs = ["android_external_texture_surface.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":default_platform_android_external_texture_surface",
        ":definitions",
        ":platform_android_external_texture_surface",
        "//core:config",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media:media_color_space",
        "//core/render",
        "//core/render:content_security_level",
        "//core/split_engine:split_engine_serializer",
        "//core/view:base_view",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ] + if_android_external_texture_surface_uses_image_reader(
        [
            ":image_reader_android_external_texture_surface",
        ],
    ),
    alwayslink = 1,
)

cc_library(
    name = "platform_android_external_texture_surface",
    hdrs = ["platform_android_external_texture_surface.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":definitions",
        "//core/common:owned_ptr",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media:media_color_space",
        "//core/render:content_security_level",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "default_platform_android_external_texture_surface",
    srcs = ["default_platform_android_external_texture_surface.cc"],
    hdrs = ["default_platform_android_external_texture_surface.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":definitions",
        ":platform_android_external_texture_surface",
        "//core:config",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media:media_color_space",
        "//core/render",
        "//core/render:content_security_level",
        "//core/view:base_view",
        "//core/view/platforms/android/wrappers:graphics",
        "//core/view/platforms/android/wrappers:surface_texture",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "image_reader_android_external_texture_surface",
    srcs = [] + if_android_external_texture_surface_uses_image_reader(
        [
            ":image_reader_android_external_texture_surface.cc",
        ],
    ),
    hdrs = [] + if_android_external_texture_surface_uses_image_reader(
        [
            ":image_reader_android_external_texture_surface.h",
        ],
    ),
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":definitions",
        ":platform_android_external_texture_surface",
        "//core/common:registry",
        "//core/common:robin_map",
        "//core/common:small_source_location",
        "//core/math",
        "//core/media:media_color_space",
        "//core/ncsb",
        "//core/render",
        "//core/render:content_security_level",
        "//core/view:base_view",
        "//core/view/platforms/android/ndkwrappers:image",
        "//core/view/platforms/android/ndkwrappers:image_reader",
        "//core/view/platforms/android/wrappers:graphics",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/container:inlined_vector",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

# --config=android_all
# --android_ndk_min_sdk_version=23

# TODO: (broken link) - The test requires --define=IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER=1
# to be enabled to pass, but it seems all the extra parameters are directed to cc_library through
# kwargs here:
# google3/third_party/impress/testing/build_defs.bzl;l=283.
# Fix this and re-enable the code.
filegroup(
    name = "deactivate_tests",
    srcs = ["image_reader_android_external_texture_surface_test.cc"],
    tags = ["ignore_srcs"],
)

# --config=android_all
# --android_ndk_min_sdk_version=28
# blaze test //core/render/android:image_reader_android_external_texture_surface_test \
#     --config=android_all --android_ndk_min_sdk_version=28 \
#     --define=IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER=1
# ImageReaderAndroidExternalTextureSurface requires API level 26 and above, but Android emulator on
# Forge requires API level 28 and above to run the test.
# cc_test_on_android(
#     name = "image_reader_android_external_texture_surface_test",
#     srcs = [
#         "image_reader_android_external_texture_surface_test.cc",
#     ],
#     args = ["--define=IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER=1"],
#     cc_deps = [
#         ":definitions",
#         ":image_reader_android_external_texture_surface",
#         ":platform_android_external_texture_surface",
#         "@mediapipe//mediapipe/framework/port:gtest_main",
#         "@com_google_absl//absl/container:flat_hash_map",
#         "//:testing_api",
#         "//testing",
#     ],
#     copts = imp_google3_copts() + ["-DIMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER"],
#     defines = imp_defines(),
#     java_deps = [
#         "//java/com/google/ar/imp/core/media:media_no_jni",
#     ],
#     linkopts = imp_default_jni_linkopts(),
#     target_device = "@bazel_tools//tools/mobile/devices/android/pixel_8:pixel_8_34_x86_64_gms_stable",
#     test_tags = [
#         "cc_android_28",
#         "requires-mem:24g",  # Necessary to run Google 34 emulator, see (broken link)
#     ],
# )

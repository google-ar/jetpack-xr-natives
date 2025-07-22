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

load(
    "//build_tools:imp.bzl",
    "if_non_android",
    "imp_default_jni_linkopts",
    "imp_google3_copts",
)

# TODO: (broken link) - Add unit tests, including MSAN.

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

filegroup(
    name = "impress_jni_srcs",
    srcs = [
        "create_view.cc",
        "impress_api.cc",
    ],
)

# A list of the common dependencies between the production and test versions of the Impress JNI
# library.
_IMPRESS_JNI_COMMON_DEPS = [
    ":asset_animator",
    ":asset_loader",
    ":jni_conversion_utils",
    ":stereo_surface",
    "//core/common:log",
    "@com_google_absl//absl/log:check",
    "@com_google_absl//absl/status",
    "@com_google_absl//absl/status:statusor",
    "@com_google_absl//absl/strings:cord",
    "@com_google_absl//absl/strings:string_view",
    "@third_party//filament/filament:filament_hdrs",
    "@third_party//filament/libs:utils_hdrs",
    "//:api",
    "//core/common:buffer_access",
    "//core/common:jni_helpers",
    "//core/input",
    "//core/material_library:fbs",
    "//core/material_library:generic_material_spec",
    "//core/math",
    "//core/media:media_color_space",
    "//core/media:media_type",
    "//core/ncsb",
    "//core/render:content_security_level",
    "//core/view/platforms/android/jni_helpers:exception_helper",
    "//core/view/platforms/android/wrappers:graphics",
    ":impress_api_view_header",
] + if_non_android(["@bazel_tools//tools/jdk:jni"])

# A list of the common dependencies between the production and test versions of the Impress API
# View.
_IMPRESS_API_VIEW_COMMON_DEPS = [
    ":asset_animator",
    ":asset_loader",
    ":asset_ptr_map",
    ":bindings_material",
    ":bindings_object",
    ":bindings_texture",
    ":stereo_surface",
    "@com_google_absl//absl/container:flat_hash_map",
    "@com_google_absl//absl/container:flat_hash_set",
    "//core/common:log",
    "@com_google_absl//absl/log:check",
    "@com_google_absl//absl/status",
    "@com_google_absl//absl/status:statusor",
    "@com_google_absl//absl/strings:string_view",
    "@third_party//filament/filament:filament_hdrs",
    "//:api",
    "//:impl_full",
    "//:platform_android",
    "//core/common:jni_helpers",
    "//core/material_library:generic_material_parameters",
    "//core/material_library:generic_material_spec",
    "//core/math",
    "//core/media:media_color_space",
    "//core/media:media_type",
    "//core/render:content_security_level",
    "//core/render/android:platform_android_external_texture_surface",
    "//core/split_engine/materials:split_engine_generic_material",
    "//core/view/framework:framework_proto_cc",
    "//core/view/platforms/android/wrappers:graphics",
    "//core/view/utils:frame_time",
    "@third_party//split_engine/materials:water_reflection_material",
    "@mediapipe//mediapipe/framework/port:status",
]

cc_library(
    name = "impress_api_view_header",
    hdrs = ["impress_api_view.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = _IMPRESS_API_VIEW_COMMON_DEPS,
    alwayslink = 1,
)

cc_library(
    name = "impress_api_view",
    srcs = ["impress_api_view.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = _IMPRESS_API_VIEW_COMMON_DEPS + [
        ":impress_api_view_header",
        "//core/assets",
        "//core/async:future",
        "//core/lighting:environment_light",
        "//core/lighting:image_based_lighting_asset",
        "//core/ncsb",
        "//core/render",
        "//core/split_engine:split_engine_serializer",
        "//core/split_engine/materials:split_engine_material",
        "//core/view/framework",
        "@com_google_absl//absl/strings:str_format",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "asset_ptr_map",
    srcs = ["asset_ptr_map.cc"],
    hdrs = ["asset_ptr_map.h"],
    copts = imp_google3_copts(),
    deps = [
        ":asset_loader",
        "//:api",
        "//core/assets",
        "//core/lighting:image_based_lighting_asset",
        "//core/split_engine:split_engine_serializer",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@re2",
    ],
    alwayslink = 1,
)

cc_library(
    name = "asset_loader",
    srcs = ["asset_loader.cc"],
    hdrs = ["asset_loader.h"],
    copts = imp_google3_copts(),
    deps = ["//core/common:jni_helpers"] + if_non_android([
        "@bazel_tools//tools/jdk:jni",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "asset_animator",
    srcs = ["asset_animator.cc"],
    hdrs = ["asset_animator.h"],
    copts = imp_google3_copts(),
    deps = ["//core/common:jni_helpers"] + if_non_android([
        "@bazel_tools//tools/jdk:jni",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "impress_jni",
    srcs = [":impress_jni_srcs"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = _IMPRESS_JNI_COMMON_DEPS + [":impress_api_view"],
    alwayslink = 1,
)

cc_library(
    name = "test_impress_jni",
    testonly = True,
    srcs = [":impress_jni_srcs"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = _IMPRESS_JNI_COMMON_DEPS + [
        ":impress_api_test_helper",
        ":test_impress_api_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "stereo_surface",
    srcs = ["stereo_surface.cc"],
    hdrs = ["stereo_surface.h"],
    copts = imp_google3_copts(),
    deps = [
        "//core/async:future",
        "//core/common:log",
        "//core/math",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/ncsb",
        "//core/render",
        "//core/render:content_security_level",
        "//core/render/android:android_external_texture_surface",
        "//core/render/android:definitions",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//split_engine/materials:jxr_media_material",
    ],
    alwayslink = 1,
)

cc_library(
    name = "bindings_texture",
    srcs = ["bindings_texture.cc"],
    hdrs = ["bindings_texture.h"],
    copts = imp_google3_copts(),
    deps = [
        ":bindings_object",
        "//core/common:small_source_location",
        "//core/render",
    ],
    alwayslink = 1,
)

cc_library(
    name = "bindings_material",
    srcs = ["bindings_material.cc"],
    hdrs = [
        "bindings_material.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":bindings_object",
        "//core/common:hash",
        "//core/common:type_traits",
        "//core/split_engine/materials:split_engine_material",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "bindings_object",
    hdrs = ["bindings_object.h"],
    copts = imp_google3_copts(),
    alwayslink = 1,
)

cc_library(
    name = "jni_conversion_utils",
    srcs = ["jni_conversion_utils.cc"],
    hdrs = ["jni_conversion_utils.h"],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:jni_helpers",
        "//core/material_library:fbs",
        "//core/material_library:generic_material_spec",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@third_party//filament/filament:filament_hdrs",
    ] + if_non_android([
        "@bazel_tools//tools/jdk:jni",
    ]),
    alwayslink = 1,
)

# Native JNI library for the Impress API Bindings. See THIRD_PARTY_NOTICES.txt for license info.
# Example build command:
# bazel build --define=IMP_INCLUDE_STEREO_VARIANT_BY_DEFAULT=1 --copt=-D_LIBCPP_ABI_NAMESPACE=__1
# --define=IMP_EMBED_ASSETS=1 --define=FILAMENT_USES_OPENGL_GLES3=1 --config=release
# --define=FILAMENT_ENABLE_MULTIVIEW=1 --define=IMP_ENABLE_STEREO_TYPE_MULTIVIEW=1
# --define=SYSTRACE=1 --config=android_arm64
cc_binary(
    name = "impress_api_jni",
    linkopts = [
        "-landroid",
        "-lbinder_ndk",
        "-Wl,--no-undefined",
    ] + imp_default_jni_linkopts(),
    linkshared = True,
    deps = [
        ":impress_jni",
        "//:platform_android",
        "//core/split_engine/android/view:split_engine_jni",
        "@third_party//split_engine:subspace_jni",
    ],
)

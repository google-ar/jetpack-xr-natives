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

load("@com_google_impress//build_tools:imp.bzl", "if_non_android", "imp_default_jni_linkopts", "imp_google3_copts")
load("//build_tools/single_jar:single_jar.bzl", "single_jar")

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
    ":jni_utils",
    ":stereo_surface",
    "@com_google_impress//core/common:log",
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
    ":asset_ptr_map",
    ":bindings_material",
    ":bindings_object",
    ":bindings_texture",
    ":stereo_surface",
    "@com_google_absl//absl/container:flat_hash_map",
    "@com_google_absl//absl/container:flat_hash_set",
    "@com_google_impress//core/common:log",
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
    "//core/split_engine/materials:split_engine_builtin_material",
    "//core/common:owned_ptr",
    "//core/common:type_traits",
    "//core/common:hash",
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
        ":generic_material_manager",
        ":generic_material_manager_interface",
        ":impress_api_view_header",
        ":model_manager",
        ":model_manager_interface",
        ":node_manager",
        ":node_manager_interface",
        ":skybox_manager",
        ":skybox_manager_interface",
        ":stereo_surface_manager",
        ":stereo_surface_manager_interface",
        ":texture_manager",
        ":texture_manager_interface",
        ":water_material_manager",
        ":water_material_manager_interface",
        "//core/assets",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/lighting:environment_light",
        "//core/lighting:image_based_lighting_asset",
        "//core/materials",
        "//core/ncsb",
        "//core/render",
        "//core/view/framework",
        "@com_google_absl//absl/strings:str_format",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "model_manager_interface",
    hdrs = ["model_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":base_asset_animator",
        ":base_asset_loader",
        "//core/math",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "model_manager",
    srcs = ["model_manager.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":asset_ptr_map",
        ":base_asset_animator",
        ":base_asset_loader",
        ":bindings_material",
        ":impress_api_view_header",
        ":model_manager_interface",
        "//apibindings/model_interaction_ux:reform_affordance_ux",
        "//core/assets",
        "//core/common:small_source_location",
        "//core/geometry/shapes",
        "//core/materials",
        "//core/ncsb",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "skybox_manager_interface",
    hdrs = ["skybox_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":base_asset_loader",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "skybox_manager",
    srcs = ["skybox_manager.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":asset_ptr_map",
        ":base_asset_loader",
        ":impress_api_view_header",
        ":skybox_manager_interface",
        "//core/assets",
        "//core/lighting:image_based_lighting_asset",
        "//core/math",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "stereo_surface_manager_interface",
    hdrs = ["stereo_surface_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//apibindings:stereo_surface",
        "//core/math",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/render:content_security_level",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "stereo_surface_manager",
    srcs = ["stereo_surface_manager.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":bindings_texture",
        ":impress_api_view_header",
        ":stereo_surface",
        ":stereo_surface_manager_interface",
        "//core/common:small_source_location",
        "//core/input",
        "//core/media:media_color_space",
        "//core/media:media_type",
        "//core/ncsb",
        "//core/render",
        "//core/render:content_security_level",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "texture_manager_interface",
    hdrs = ["texture_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":base_asset_loader",
        "//core/assets",
        "//core/lighting:image_based_lighting_asset",
        "//core/render",
        "//core/view/utils:asset",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "texture_manager",
    srcs = ["texture_manager.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":asset_ptr_map",
        ":base_asset_loader",
        ":bindings_texture",
        ":impress_api_view_header",
        ":texture_manager_interface",
        "//core/assets",
        "//core/async:future",
        "//core/common:small_source_location",
        "//core/lighting:environment_light",
        "//core/lighting:image_based_lighting_asset",
        "//core/render",
        "//core/view/framework",
        "//core/view/utils:asset",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "generic_material_manager_interface",
    hdrs = ["generic_material_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":base_asset_loader",
        "//core/material_library:generic_material_spec",
        "//core/math",
        "@com_google_absl//absl/status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "generic_material_manager",
    srcs = ["generic_material_manager.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":base_asset_loader",
        ":bindings_material",
        ":generic_material_manager_interface",
        ":impress_api_view_header",
        ":texture_manager_interface",
        "//core/async:future",
        "//core/common:hash",
        "//core/common:owned_ptr",
        "//core/common:small_source_location",
        "//core/common:type_traits",
        "//core/material_library:generic_material_parameters",
        "//core/material_library:generic_material_spec",
        "//core/math",
        "//core/render",
        "//core/split_engine/materials:split_engine_builtin_material",
        "//core/split_engine/materials:split_engine_generic_material",
        "//core/split_engine/materials/builtin:builtin_generic_material",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "water_material_manager_interface",
    hdrs = ["water_material_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":base_asset_loader",
        "@com_google_absl//absl/status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "water_material_manager",
    srcs = ["water_material_manager.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":base_asset_loader",
        ":bindings_material",
        ":impress_api_view_header",
        ":texture_manager_interface",
        ":water_material_manager_interface",
        "//core/async:future",
        "//core/common:hash",
        "//core/common:owned_ptr",
        "//core/common:small_source_location",
        "//core/common:type_traits",
        "//core/render",
        "//core/split_engine/materials:split_engine_builtin_material",
        "//core/split_engine/materials/builtin:builtin_water_material",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//split_engine/materials:water_reflection_material",
    ],
    alwayslink = 1,
)

cc_library(
    name = "node_manager_interface",
    hdrs = ["node_manager.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//core/math",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "node_manager",
    srcs = ["node_manager.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":impress_api_view_header",
        ":model_manager_interface",
        ":node_manager_interface",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
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
        ":base_asset_loader",
        "//:api",
        "//core/assets",
        "//core/lighting:image_based_lighting_asset",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:asset",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@re2",
    ],
    alwayslink = 1,
)

cc_library(
    name = "asset_loader",
    srcs = ["asset_loader.cc"],
    hdrs = ["asset_loader.h"],
    copts = imp_google3_copts(),
    deps = [
        ":base_asset_loader",
        "//core/common:jni_helpers",
    ] + if_non_android([
        "@bazel_tools//tools/jdk:jni",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "base_asset_loader",
    hdrs = ["base_asset_loader.h"],
    alwayslink = 1,
)

cc_library(
    name = "asset_animator",
    srcs = ["asset_animator.cc"],
    hdrs = ["asset_animator.h"],
    copts = imp_google3_copts(),
    deps = [
        ":base_asset_animator",
        "//core/common:jni_helpers",
    ] + if_non_android([
        "@bazel_tools//tools/jdk:jni",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "base_asset_animator",
    hdrs = ["base_asset_animator.h"],
    alwayslink = 1,
)

cc_library(
    name = "impress_jni",
    srcs = [":impress_jni_srcs"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = _IMPRESS_JNI_COMMON_DEPS + [
        ":generic_material_manager",
        ":generic_material_manager_interface",
        ":impress_api_view",
        ":model_manager",
        ":model_manager_interface",
        ":node_manager",
        ":node_manager_interface",
        ":skybox_manager",
        ":skybox_manager_interface",
        ":stereo_surface_manager",
        ":stereo_surface_manager_interface",
        ":texture_manager",
        ":texture_manager_interface",
        ":water_material_manager",
        ":water_material_manager_interface",
    ],
    alwayslink = 1,
)

# TODO: Move API bindings test targets to a testing subdirectory.
cc_library(
    name = "test_impress_jni",
    testonly = True,
    srcs = ["impress_api.cc"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = _IMPRESS_JNI_COMMON_DEPS + [
        ":generic_material_manager_interface",
        ":impress_api_view",
        ":model_manager_interface",
        ":node_manager_interface",
        ":skybox_manager_interface",
        ":stereo_surface_manager_interface",
        ":texture_manager_interface",
        ":water_material_manager_interface",
        "//apibindings/testing/marshalling:impress_api_test_helper",
        "//apibindings/testing/marshalling:test_create_view_factory",
        "//apibindings/testing/marshalling:test_impress_api_view",
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
        "//core/view/framework:framework_proto_cc",
        "//core/view/platforms/android/wrappers:graphics",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
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
        "//core/assets",
        "//core/common:small_source_location",
        "//core/lighting:image_based_lighting_asset",
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
        "//core/common:small_source_location",
        "//core/materials",
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

cc_library(
    name = "jni_utils",
    srcs = ["jni_utils.cc"],
    hdrs = ["jni_utils.h"],
    deps = [
        ":stereo_surface",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

# Generates a .jar containing Impress Java API bindings, Split Engine code, and Impress Java
# platform code. See THIRD_PARTY_NOTICES.txt for license info.
# Note: This JAR calls native code provided by the //apibindings:impress_api_jni
# target (.so) which must be available at runtime when using this .jar.
single_jar(
    name = "impress_no_native_lib_jar",
    # These .jar targets are the implicit outputs of android_library targets and, in Bazel,
    # kt_android_library targets.
    # TODO Modify single_jar to consume plain targets instead of implicitly-generated
    # .jar targets.
    # LINT.IfChange(jar_deps)
    jars = [
        "//java/com/google/ar/imp/view:libview.jar",
        "//java/com/google/ar/imp/view/splitengine:libsplitengine.jar",
        "//java/com/google/ar/imp/view/input:libinput_manager.jar",
        "//java/com/google/ar/imp/view:libimp_api_scuba.jar",
        "//java/com/google/ar/imp/view:libframe_scheduler.jar",
        "//java/com/google/ar/imp/view:libchoreographer_holder.jar",
        "@third_party//split_engine/java/com/google/androidxr/splitengine:libsplit_engine_subspace_manager.jar",
        "//core/split_engine/android/extensions/java/com/google/imp/splitengine/extensions:libextensions_java.jar",
        "@third_party//split_engine/java/com/google/androidxr/splitengine:libsubspace_node.jar",
        "@third_party//split_engine/java/com/google/androidxr/splitengine:libsplit_engine_input_event.jar",
        "@third_party//split_engine/java/com/google/androidxr/splitengine:libscene_leash_node_factory.jar",
    ],
    # LINT.ThenChange(//depot/google3/third_party/impress/apibindings:must_depend_jar)
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

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
    "@com_github_google_flatbuffers//:build_defs.bzl",
    "flatbuffer_cc_library",
)
load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "IMP_FLATBUFFER_INCLUDE_PATHS", "IMP_FLATC_ARGS", "imp_copts")
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

schemas = [
    "schemas/gltf_animation.fbs",
]

filegroup(
    name = "schemas",
    srcs = schemas,
)

flatbuffer_cc_library(
    name = "fbs",
    srcs = schemas,
    flatc_args = IMP_FLATC_ARGS,
    include_paths = IMP_FLATBUFFER_INCLUDE_PATHS,
    includes = ["//core/common:schemas"],
    out_prefix = "schemas/",
)

cc_library(
    name = "fbs_deps",
    deps = [
        ":fbs",
        "//core/common:fbs_deps",
    ],
    alwayslink = 1,
)

cc_library(
    name = "ease_curves",
    srcs = ["ease_curves.cc"],
    hdrs = ["ease_curves.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":scalar_animation",
        "//core/math",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "easing_functions",
    srcs = ["easing_functions.cc"],
    hdrs = ["easing_functions.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "curve",
    srcs = [
        "curve.cc",
    ],
    hdrs = [
        "curve.h",
        "curve_values.h",
        "curve_variant_helper.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":easing_functions",
        ":fbs",
        "//core/common:typed_ids",
        "//core/math",
        "//core/math:flatbuffer_support",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/types:variant",
    ],
    alwayslink = 1,
)

cc_library(
    name = "morph_target_animation",
    srcs = [
        "morph_target_animation.cc",
    ],
    hdrs = [
        "morph_target_animation.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":curve",
        ":fbs",
        "//core/common",
        "//core/common:filament_helpers",
        "//core/common:flatbuffer_helpers",
        "//core/math",
        "//core/math:flatbuffer_support",
        "//core/model",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "scalar_animation",
    srcs = [
        "scalar_animation.cc",
    ],
    hdrs = [
        "scalar_animation.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":curve",
        "//core/math",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "gltf_node_animation",
    srcs = [
        "gltf_node_animation.cc",
    ],
    hdrs = [
        "gltf_node_animation.h",
    ],
    copts = imp_copts(),
    deps = [
        ":curve",
        ":fbs",
        "//core/common:filament_helpers",
        "//core/math",
        "//core/math:flatbuffer_support",
        "@com_google_absl//absl/status:statusor",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "texture_transform_animation",
    srcs = [
        "texture_transform_animation.cc",
    ],
    hdrs = [
        "texture_transform_animation.h",
    ],
    copts = imp_copts(),
    deps = [
        ":curve",
        ":fbs",
        "//core/loader/provider/gltf:texturable_parameter",
        "//core/math",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:optional",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "material_animation",
    srcs = [
        "material_animation.cc",
    ],
    hdrs = [
        "material_animation.h",
    ],
    copts = imp_copts(),
    deps = [
        ":curve",
        ":fbs",
        ":texture_transform_animation",
        "//core/math",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:optional",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "light_animation",
    srcs = [
        "light_punctual_animation.cc",
    ],
    hdrs = [
        "light_punctual_animation.h",
    ],
    copts = imp_copts(),
    deps = [
        ":curve",
        ":fbs",
        "//core/math",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:optional",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "gltf_animation",
    srcs = [
        "gltf_animation.cc",
    ],
    hdrs = [
        "gltf_animation.h",
    ],
    copts = imp_copts(),
    deps = [
        ":fbs",
        ":gltf_node_animation",
        ":light_animation",
        ":material_animation",
        ":morph_target_animation",
        "//core/common:flatbuffer_helpers",
        "//core/common:robin_map",
        "//core/common:typed_ids",
        "//core/math",
        "//core/model",
        "//core/model:model_data_details",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "property_animator",
    srcs = [
        "property_animator.cc",
    ],
    hdrs = [
        "property_animator.h",
    ],
    copts = imp_copts(),
    deps = [
        ":curve",
        ":property_animator_state_imp_cc_proto",
        "//core/async:future",
        "//core/common:robin_map",
        "//core/math",
        "//core/math:math_proto_cc",
        "//core/ncsb",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

proto_library(
    name = "property_animator_state_proto",
    srcs = ["property_animator_state.proto"],
    deps = [
        "//core/view/framework:framework_proto",
    ],
)

imp_cc_proto_library(
    name = "property_animator_state_imp_cc_proto",
    proto = ":property_animator_state_proto",
    deps = [
        "//core/view/framework:framework_proto_cc",
    ],
)

cc_library(
    name = "gltf_conversions",
    srcs = [
        "gltf_conversions.cc",
        "gltf_conversions_animation_pointer.cc",
        "gltf_conversions_helper.cc",
    ],
    hdrs = [
        "gltf_conversions.h",
        "gltf_conversions_animation_pointer.h",
        "gltf_conversions_helper.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":fbs",
        "//core/common",
        "//core/common:data_helpers",
        "//core/common:flatbuffer_helpers",
        "//core/common:typed_ids",
        "//core/loader/provider/gltf:accessor_reader",
        "//core/loader/provider/gltf:dense_data_access",
        "//core/loader/provider/gltf:gltf_extension_names",
        "//core/loader/provider/gltf:gltf_lookup",
        "//core/loader/provider/gltf:imp_cc_schema",
        "//core/loader/provider/gltf:texturable_parameter",
        "//core/math",
        "//core/model:model_data_details",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/libs:filabridge_hdrs",
    ],
    alwayslink = 1,
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/*.png"]),
)

imp_assets(
    name = "test_data",
    srcs = [
        "test_data/AnimateAllTheThings.glb",
        "test_data/MorphStressTest.glb",
    ],
    namespace = "imp::test_data",
)

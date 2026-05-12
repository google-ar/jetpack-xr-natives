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

load("@com_google_sandboxed_api//sandboxed_api/bazel:embed_data.bzl", "sapi_cc_embed_data")

# Placeholder: load py_test
load(
    "@flatbuffers//:build_defs.bzl",
    "flatbuffer_cc_library",
)
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "IMP_FLATBUFFER_INCLUDE_PATHS", "IMP_FLATC_ARGS", "if_android", "if_dev_runtime", "if_non_android", "imp_copts", "imp_defines", "imp_google3_copts")
load("//core/loader/data:generic_material.bzl", "process_and_build_material")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

schemas = [
    "schemas/math.fbs",
    "schemas/render.fbs",
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
    out_prefix = "schemas/",
)

cc_library(
    name = "fbs_deps",
    deps = [
        ":fbs",
        "//core/math",
        "//core/math:flatbuffer_support",
    ],
    alwayslink = 1,
)

cc_library(
    name = "bit_flag",
    hdrs = ["bit_flag.h"],
    copts = imp_copts(),
    alwayslink = 1,
)

cc_library(
    name = "buffer_access",
    srcs = ["buffer_access.cc"],
    hdrs = ["buffer_access.h"],
    copts = imp_copts(),
    deps = [
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

process_and_build_material(
    name = "debug_draw_unlit",
    template_file = "debug_draw_unlit.mat",
)

process_and_build_material(
    name = "debug_draw_unlit_screenspace",
    template_file = "debug_draw_unlit_screenspace.mat",
)

filegroup(
    name = "debug_draw_materials",
    srcs = [
        ":debug_draw_unlit.cmat",
        ":debug_draw_unlit_screenspace.cmat",
    ],
)

sapi_cc_embed_data(
    name = "debug_draw_resources",
    srcs = [":debug_draw_materials"],
    flatten = True,
)

cc_library(
    name = "debug_draw",
    srcs = ["debug_draw.cc"],
    hdrs = ["debug_draw.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":common",
        ":debug_draw_resources",
        ":filament_engine_helpers",
        ":filament_helpers",
        "//core/collision:collision_flags",
        "//core/geometry/shapes",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "//core/render:render_order_constants",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@robin_map",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:filabridge_hdrs",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

filegroup(
    name = "scuba_goldens",
    srcs = glob(["scuba_goldens/**/*.png"]),
)

cc_library(
    name = "hash",
    hdrs = ["hash.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "jni_context",
    srcs = [
        "jni_context.cc",
    ],
    hdrs = [
        "jni_context.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":platform_helpers",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "jni_helpers",
    srcs = ["jni_helpers.cc"],
    hdrs = ["jni_helpers.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":common",
        ":context",
        ":jni_context",
        ":typed_ids",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/types:variant",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "jni_proto_helpers",
    hdrs = ["jni_proto_helpers.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":context",
        "//core/proto:proto_reader",
        "@com_google_absl//absl/strings:string_view",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "registry",
    srcs = ["registry.cc"],
    hdrs = ["registry.h"],
    copts = imp_copts(),
    deps = [
        ":hash",
        ":type_traits",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "template_helpers",
    hdrs = ["template_helpers.h"],
    copts = imp_google3_copts(),
    alwayslink = 1,
)

cc_library(
    name = "type_helpers",
    hdrs = ["type_helpers.h"],
    copts = imp_copts(),
    alwayslink = 1,
)

cc_library(
    name = "test_helpers",
    testonly = 1,
    srcs = ["test_helpers.cc"],
    hdrs = ["test_helpers.h"],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":common",
        "@mediapipe//mediapipe/framework/port:gtest",
    ],
    alwayslink = 1,
)

cc_library(
    name = "jvm_test_helpers",
    testonly = 1,
    srcs = ["jvm_test_helpers.cc"],
    hdrs = ["jvm_test_helpers.h"],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        "//devtools/java/build:java_locations",
        "//util/java:jvm",
        "@com_google_absl//absl/flags:flag",
        "@mediapipe//mediapipe/framework/port:gtest",
    ],
    alwayslink = 1,
)

cc_library(
    name = "context",
    srcs = ["context.cc"],
    hdrs = ["context.h"],
    copts = imp_google3_copts(),
    deps = [
        ":jni_context",
        "//core:config",
        "@com_google_absl//absl/types:span",
    ] + if_non_android(
        [
            "@bazel_tools//tools/jdk:jni",
        ],
    ),
    alwayslink = 1,
)

cc_library(
    name = "owned_ptr",
    srcs = [
        "owned_ptr_utilities.cc",
    ],
    hdrs = [
        "owned_ptr.h",
        "owned_ptr_traits.h",
        "owned_ptr_utilities.h",
    ],
    copts = imp_copts(),
    deps = [
        ":ref_counter",
        ":small_source_location",
        ":type_traits",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:log_severity",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "small_source_location",
    hdrs = [
        "small_source_location.h",
    ],
    copts = imp_copts(),
    alwayslink = 1,
)

py_test(
    name = "owned_ptr_nc_test",
    size = "large",
    srcs = ["owned_ptr_nc_test.py"],
    data = ["owned_ptr_nc"],
    tags = ["non_compile_test"],
    deps = [
        "//testing/pybase",
        "//testing/pybase:fake_target_util",
        "//third_party/py/google/protobuf:use_fast_cpp_protos",  # Automatically added (broken link)
    ],
)

cc_library(
    name = "owned_or_borrowed_ptr",
    hdrs = [
        "owned_or_borrowed_ptr.h",
    ],
    copts = imp_copts(),
    deps = [
        ":owned_ptr",
        ":small_source_location",
    ],
    alwayslink = 1,
)

cc_library(
    name = "smooth",
    hdrs = [
        "smooth.h",
    ],
    copts = imp_copts(),
    #linkstatic = 1,
    deps = [
        ":common",
        "@third_party//filament/libs:math_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "robin_map",
    hdrs = [
        "robin_map.h",
        "robin_set.h",
    ],
    copts = imp_copts(),
    deps = [
        "@com_google_absl//absl/hash",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "visit",
    hdrs = [
        "visit.h",
    ],
    copts = imp_copts(),
    deps = [
        "@com_google_absl//absl/types:variant",
    ],
    alwayslink = 1,
)

cc_library(
    name = "resource_helpers",
    srcs = [
        "resource_helpers.cc",
    ],
    hdrs = [
        "resource_helpers.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":buffer_access",
        ":optional_error",
        ":platform_helpers",
        ":robin_map",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:cord",
        "@com_google_impress//core/common:log",
        "@com_google_sandboxed_api//sandboxed_api:embed_file",
    ],
    alwayslink = 1,
)

cc_library(
    name = "filament_engine_helpers",
    srcs = [
        "filament_engine_helpers.cc",
    ],
    hdrs = [
        "filament_engine_helpers.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":buffer_access",
        ":optional_error",
        ":platform_helpers",
        ":resource_helpers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status:statusor",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "filament_helpers",
    srcs = [
        "filament_helpers.cc",
    ],
    hdrs = [
        "filament_helpers.h",
    ],
    copts = imp_copts(),
    deps = [
        "//core/geometry:geometry_helper",
        "//core/geometry/shapes",
        "//core/math",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "common",
    srcs = [
        "file_helpers.cc",
        "string_helpers.cc",
    ],
    hdrs = [
        "buffer_access.h",
        "enum_flags.h",
        "file_helpers.h",
        "optional_error.h",
        "platform_helpers.h",
        "resource_helpers.h",
        "string_helpers.h",
        "visit.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":buffer_access",
        ":optional_error",
        ":platform_helpers",
        ":resource_helpers",
        ":visit",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/types:variant",
    ],
    alwayslink = 1,
)

cc_library(
    name = "status_helpers",
    hdrs = [
        "status_helpers.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = ["@com_google_absl//absl/status:statusor"],
    alwayslink = 1,
)

cc_library(
    name = "type_traits",
    hdrs = [
        "type_traits.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":hash",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "imp_matchers",
    testonly = 1,
    hdrs = [
        "imp_matchers.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        "//core/math",
        "@mediapipe//mediapipe/framework/port:gtest",
        "@third_party//filament/libs:math_io",
    ],
    alwayslink = 1,
)

cc_library(
    name = "optional_error",
    srcs = ["optional_error.cc"],
    hdrs = ["optional_error.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":string_helpers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:optional",
    ],
    alwayslink = 1,
)

cc_library(
    name = "enum_flags",
    hdrs = ["enum_flags.h"],
    copts = imp_copts(),
    linkstatic = 1,
    alwayslink = 1,
)

cc_library(
    name = "string_helpers",
    hdrs = ["string_helpers.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
    alwayslink = 1,
)

cc_library(
    name = "data_helpers",
    hdrs = ["data_helpers.h"],
    copts = imp_copts(),
    linkstatic = 1,
    alwayslink = 1,
)

cc_library(
    name = "platform_helpers",
    srcs = ["platform_helpers.cc"],
    hdrs = ["platform_helpers.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = 1,
    deps = [
        ":string_helpers",
        "//core:config",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/base:log_severity",
        "@com_google_absl//absl/log:log_entry",
        "@com_google_absl//absl/log:log_sink",
        "@com_google_absl//absl/log:log_sink_registry",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
    ],
    alwayslink = 1,
)

cc_library(
    name = "trace",
    srcs = ["trace.cc"],
    hdrs = [
        "trace.h",
        "trace_details.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":type_traits",
        "//core:config",
        "@com_google_absl//absl/strings",
        "@third_party//filament/libs:utils_hdrs",
    ] + if_dev_runtime([
        "//core/performance:profiler",
    ]) + select({
        "//core:imp_perfetto_enabled": [
            "@com_google_impress//core/common:log",
            "@perfetto//:libperfetto_client_experimental",
        ],
        "//conditions:default": [],
    }),
    alwayslink = 1,
)

cc_library(
    name = "typed_ids",
    srcs = ["typed_tree.cc"],
    hdrs = [
        "paired_span.h",
        "paired_vector.h",
        "typed_container_helpers.h",
        "typed_id.h",
        "typed_span.h",
        "typed_tree.h",
        "typed_vector.h",
    ],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":data_helpers",
        ":platform_helpers",
        ":type_traits",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/container:inlined_vector",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/meta:type_traits",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "typed_set_vector",
    hdrs = ["typed_set_vector.h"],
    copts = imp_google3_copts(),
    deps = [
        ":typed_ids",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@third_party//filament/libs:utils",  # build_cleaner: keep
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "variant",
    srcs = ["variant.h"],
    hdrs = [
        "variant.h",
    ],
    copts = imp_copts(),
    deps = [
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "invocable",
    hdrs = ["invocable.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [":type_erased"],
    alwayslink = 1,
)

cc_library(
    name = "holdable",
    hdrs = ["holdable.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":type_erased",
    ],
    alwayslink = 1,
)

cc_library(
    name = "bit_vector",
    hdrs = ["bit_vector.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        "@com_google_absl//absl/types:span",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "flatbuffer_helpers",
    srcs = ["flatbuffer_helpers.cc"],
    hdrs = ["flatbuffer_helpers.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":buffer_access",
        ":fbs",
        ":optional_error",
        ":typed_ids",
        "//core/math",
        "//core/math:flatbuffer_support",
        "@com_google_absl//absl/types:span",
        "@flatbuffers",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "ref_counter",
    srcs = ["ref_counter.cc"],
    hdrs = ["ref_counter.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":small_source_location",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
    ],
    alwayslink = 1,
)

# TODO: Add Platform specific implementation for iOS.
cc_library(
    name = "zip_helpers",
    srcs = if_android(
        ["zip_helpers_android.cc"],
        ["zip_helpers.cc"],
    ),
    hdrs = ["zip_helpers.h"],
    copts = imp_copts(),
    linkstatic = 1,
    deps = [
        ":buffer_access",
        ":optional_error",
        ":platform_helpers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@mediapipe//mediapipe/framework/port:status",
    ] + if_android(
        [
            "//core/common:jni_helpers",
            ":jni_context",
        ],
        ["@zlib_imp//:zlib"],
    ),
    alwayslink = 1,
)

cc_library(
    name = "tuple_helpers",
    hdrs = ["tuple_helpers.h"],
    copts = imp_copts(),
    linkstatic = 1,
    alwayslink = 1,
)

cc_library(
    name = "crtp_helper",
    hdrs = ["crtp_helper.h"],
    copts = imp_copts(),
    linkstatic = 1,
    alwayslink = 1,
)

cc_library(
    name = "rememberer",
    srcs = [
        "rememberer.cc",
    ],
    hdrs = [
        "rememberer.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":holdable",
        ":invocable",
        "//core/async:executor",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/synchronization",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "pass_key",
    hdrs = ["pass_key.h"],
    copts = imp_copts(),
    alwayslink = 1,
)

cc_library(
    name = "enum_indexed_array",
    hdrs = ["enum_indexed_array.h"],
    copts = imp_copts(),
    alwayslink = 1,
)

cc_library(
    name = "owned_or_unowned_memory",
    hdrs = ["owned_or_unowned_memory.h"],
    copts = imp_copts(),
    deps = [
        ":platform_helpers",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "type_erased",
    hdrs = ["type_erased.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":type_traits",
        "@com_google_absl//absl/log:check",
    ],
    alwayslink = 1,
)

# Due to issues with Copybara + Bazel + AOSP, we need to use this logging redirection helper to
# switch from absl logging to impress logging.
cc_library(
    name = "log",
    srcs = ["log.cc"],
    hdrs = ["log.h"],
    linkstatic = True,
    deps = [
        ":platform_helpers",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "entity_absl_hasher",
    hdrs = ["entity_absl_hasher.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        "@com_google_absl//absl/hash",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "vector_helpers",
    hdrs = ["vector_helpers.h"],
    copts = imp_copts(),
    linkstatic = True,
    alwayslink = 1,
)

py_test(
    name = "typed_container_nc_test",
    size = "large",
    srcs = ["typed_container_nc_test.py"],
    data = ["typed_container_test_nc"],
    tags = ["non_compile_test"],
    deps = [
        "//testing/pybase",
        "//testing/pybase:fake_target_util",
        "//third_party/py/google/protobuf:use_fast_cpp_protos",  # Automatically added (broken link)
    ],
)

py_test(
    name = "typed_set_vector_nc_test",
    size = "large",
    srcs = ["typed_set_vector_nc_test.py"],
    data = ["typed_set_vector_test_fake_binary"],
    tags = ["non_compile_test"],
    deps = [
        "//testing/pybase",
        "//testing/pybase:fake_target_util",
        "//third_party/py/google/protobuf:use_fast_cpp_protos",  # Automatically added (broken link)
    ],
)

cc_library(
    name = "allocator_pool",
    hdrs = [
        "allocator_pool.h",
    ],
    copts = imp_copts(),
    defines = imp_defines(),
    deps = [
        ":invocable",
        "//core/view/utils:macros",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "copyable_ptr",
    hdrs = ["copyable_ptr.h"],
    copts = imp_copts(),
    linkstatic = True,
    alwayslink = 1,
)

cc_library(
    name = "optional_with_default",
    hdrs = ["optional_with_default.h"],
    copts = imp_copts(),
    deps = [
        "@com_google_absl//absl/base:no_destructor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "filament_status_helpers",
    srcs = ["filament_status_helpers.cc"],
    hdrs = ["filament_status_helpers.h"],
    copts = imp_copts(),
    deps = [
        "@com_google_absl//absl/status",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

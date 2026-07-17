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

# This package defines Impress proto generator and runtime library.
# See (broken link) for documentation, user guide and examples.

load("@bazel_skylib//:bzl_library.bzl", "bzl_library")
load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//build_tools:imp.bzl", "if_dev_runtime", "imp_copts")
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

proto_library(
    name = "imp_proto",
    srcs = ["imp.proto"],
    deps = [
        ":imp_editor_proto",
        "@com_google_protobuf//:descriptor_proto",
    ],
)

cc_proto_library(
    name = "imp_cc_proto",
    deps = [":imp_proto"],
)

proto_library(
    name = "imp_editions_proto",
    srcs = ["imp_editions.proto"],
    deps = [
        ":imp_proto",
        "@com_google_protobuf//:descriptor_proto",
    ],
)

cc_proto_library(
    name = "imp_editions_cc_proto",
    deps = [":imp_editions_proto"],
)

proto_library(
    name = "imp_editor_proto",
    srcs = ["imp_editor.proto"],
)

cc_proto_library(
    name = "imp_editor_cc_proto",
    deps = [":imp_editor_proto"],
)

imp_cc_proto_library(
    name = "imp_editor_imp_cc_proto",
    editor_proto_libs = [],
    proto = ":imp_editor_proto",
)

cc_binary(
    name = "proto-gen-imp-cc",
    srcs = [
        "proto-gen-imp-cc.cc",
    ],
    data = ["@com_google_protobuf//:protoc"],
    deps = [
        ":imp_cc_proto",
        ":imp_editions_cc_proto",
        ":imp_editor_cc_proto",
        "//core/common:hash",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_protobuf//:protobuf",
        "@com_google_protobuf//:protoc_lib",
        "@com_google_protobuf//src/google/protobuf/compiler:code_generator",
        "@mediapipe//mediapipe/framework/port:status",
    ],
)

cc_library(
    name = "proto_common",
    hdrs = ["proto_common.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":proto_traits",
        "//core/common:copyable_ptr",
        "//core/common:hash",
        "//core/common:type_traits",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

cc_library(
    name = "proto_stringify",
    hdrs = ["proto_stringify.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        "//core:config",
    ] + if_dev_runtime([
        ":textproto_writer",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "proto_reader",
    srcs = [
        "parse_message_visitor.cc",
        "proto_reader.cc",
    ],
    hdrs = [
        "parse_message_visitor.h",
        "proto_reader.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":any_imp_proto",
        ":proto_common",
        "//core/common:copyable_ptr",
        "//core/common:hash",
        "//core/common:optional_with_default",
        "//core/common:platform_helpers",
        "//core/common:robin_map",
        "//core/common:type_traits",
        "@boost_beast",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "proto_writer",
    srcs = ["proto_writer.cc"],
    hdrs = ["proto_writer.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":any_imp_proto",
        ":proto_common",
        "//core/common:copyable_ptr",
        "//core/common:one_of",
        "//core/common:optional_with_default",
        "//core/common:template_helpers",
        "@boost_beast",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "json_reader",
    srcs = [
        "json_message_visitor.cc",
        "json_reader.cc",
    ],
    hdrs = [
        "json_message_visitor.h",
        "json_reader.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":any_imp_proto",
        ":proto_common",
        ":proto_writer",
        "//core/common:copyable_ptr",
        "//core/common:hash",
        "//core/common:optional_with_default",
        "//core/common:robin_map",
        "//core/common:type_traits",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "json_writer",
    srcs = ["json_writer.cc"],
    hdrs = ["json_writer.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":any_imp_proto",
        ":proto_common",
        ":proto_reader",
        "//core/common:copyable_ptr",
        "//core/common:one_of",
        "//core/common:optional_with_default",
        "//core/common:template_helpers",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
    ],
    alwayslink = 1,
)

cc_library(
    name = "textproto_reader",
    srcs = ["textproto_reader.cc"],
    hdrs = ["textproto_reader.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":any_imp_proto",
        ":proto_common",
        ":proto_writer",
        "//core/common:copyable_ptr",
        "//core/common:hash",
        "//core/common:optional_with_default",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "textproto_writer",
    srcs = ["textproto_writer.cc"],
    hdrs = ["textproto_writer.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":any_imp_proto",
        ":proto_common",
        ":proto_reader",
        "//core/common:bit_flag",
        "//core/common:copyable_ptr",
        "//core/common:one_of",
        "//core/common:optional_with_default",
        "//core/common:platform_helpers",
        "//core/common:template_helpers",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
    ],
    alwayslink = 1,
)

cc_library(
    name = "proto_differ",
    srcs = ["proto_differ.cc"],
    hdrs = ["proto_differ.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":proto_common",
        "//core/common:copyable_ptr",
        "//core/common:one_of",
        "//core/common:optional_with_default",
        "//core/common:template_helpers",
        "//core/math",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
    ],
    alwayslink = 1,
)

cc_library(
    name = "proto_matchers",
    testonly = True,
    srcs = ["proto_matcher_utils.cc"],
    hdrs = [
        "proto_matcher_utils.h",
        "proto_matchers.h",
    ],
    deps = [
        ":any_imp_proto",
        ":proto_common",
        ":proto_reader",
        "//core/common:copyable_ptr",
        "//core/view/utils:string_map",
        "//third_party/jsoncpp:json_compat",
        "//third_party/protobuf/util:json_util",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@jsoncpp",
        "@mediapipe//mediapipe/framework/port:gtest",
    ],
    alwayslink = 1,
)

cc_library(
    name = "proto_traits",
    hdrs = ["proto_traits.h"],
    deps = [
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

proto_library(
    name = "any_proto",
    srcs = ["any.proto"],
    exports = [
        "@com_google_protobuf//:any_proto",
    ],
    deps = ["@com_google_protobuf//:any_proto"],
)

imp_cc_proto_library(
    name = "any_imp_proto",
    proto = ":any_proto",
    # :proto_stringify transitively depends on any.proto. We disable it for this proto and just
    # print out the basic output instead.
    stringify_proto_libs = [],
)

cc_library(
    name = "test_native_h",
    hdrs = ["test_native.h"],
    deps = [":proto_common"],
    alwayslink = 1,
)

cc_library(
    name = "test_include_one",
    hdrs = ["test_include_one.h"],
    alwayslink = 1,
)

cc_library(
    name = "test_include_two",
    hdrs = ["test_include_two.h"],
    alwayslink = 1,
)

proto_library(
    name = "unused_import_proto",
    srcs = ["unused_import.proto"],
    deps = [
        "@com_google_protobuf//:descriptor_proto",
    ],
)

proto_library(
    name = "test_proto",
    srcs = [
        "test.proto",
        "test_edition_2023.proto",
        "test_edition_2024.proto",
    ],
    option_deps = [":imp_proto"],
    deps = [
        ":imp_editions_proto",
        ":imp_proto",
        ":test_standard_proto",
        ":unused_import_proto",
        "@com_google_protobuf//:any_proto",
    ],
)

imp_cc_proto_library(
    name = "test_imp_proto",
    proto = ":test_proto",
    deps = [
        ":any_imp_proto",
        ":test_include_one",
        ":test_include_two",
        ":test_native_h",
        ":test_standard_cc_proto",
    ],
)

cc_proto_library(
    name = "test_cc_proto",
    deps = [":test_proto"],
)

proto_library(
    name = "test_standard_proto",
    srcs = ["test_standard.proto"],
)

cc_proto_library(
    name = "test_standard_cc_proto",
    deps = [":test_standard_proto"],
)

# Run subset of proto tests in dev runtime.

proto_library(
    name = "duration_proto",
    srcs = ["duration.proto"],
    exports = ["//google/protobuf:duration"],
    deps = ["//google/protobuf:duration"],
)

imp_cc_proto_library(
    name = "duration_imp_proto",
    proto = ":duration_proto",
)

cc_library(
    name = "duration_utils",
    srcs = ["duration_utils.cc"],
    hdrs = ["duration_utils.h"],
    deps = [
        ":duration_imp_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
    alwayslink = 1,
)

proto_library(
    name = "fuzz_proto",
    srcs = ["fuzz.proto"],
    deps = [":imp_proto"],
)

imp_cc_proto_library(
    name = "fuzz_imp_proto",
    proto = ":fuzz_proto",
)

bzl_library(
    name = "imp_cc_proto_library_bzl",
    srcs = ["imp_cc_proto_library.bzl"],
    deps = [
        "//build_tools:path_tools_bzl",
        "@bazel_skylib//lib:paths",
        "@bazel_tools//tools/cpp:toolchain_utils",
        "@rules_cc//cc/common",
    ],
)

bzl_library(
    name = "imp_encode_proto_bzl",
    srcs = ["imp_encode_proto.bzl"],
    deps = [
        "@bazel_skylib//lib:paths",
        "@mediapipe//mediapipe/framework:encode_binary_proto_bzl",
    ],
)

bzl_library(
    name = "imp_isf_data_bzl",
    srcs = ["imp_isf_data.bzl"],
    deps = [
        ":imp_encode_proto_bzl",
    ],
)

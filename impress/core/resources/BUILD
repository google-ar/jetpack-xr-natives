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

load("@bazel_skylib//:bzl_library.bzl", "bzl_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:objc_library.bzl", "objc_library")
load("//build_tools:imp.bzl", "if_dev_runtime", "if_non_android", "imp_google3_copts")
load("//core/resources:resources.bzl", "imp_resources")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

exports_files([
    "resource_h.tmpl",
    "resource_embedded_cc.tmpl",
    "resource_remote_cc.tmpl",
])

cc_library(
    name = "url_loader",
    srcs = ["url_loader.cc"],
    hdrs = [
        "url_loader.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":download_progress_info",
        "//core/async:future",
        "//core/async:future_interrupter",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
    ],
    alwayslink = 1,
)

cc_library(
    name = "download_progress_info",
    hdrs = [
        "download_progress_info.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:robin_map",
        "@com_google_absl//absl/synchronization",
    ],
    alwayslink = 1,
)

cc_library(
    name = "android_resource_loader",
    srcs = [
        "android_resource_loader.cc",
    ],
    hdrs = [
        "android_resource_loader.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":url_loader",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/view/platforms/android/wrappers:activity_context",
        "//core/view/platforms/android/wrappers:input_stream",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "android_url_loader",
    srcs = [
        "android_url_loader.cc",
    ],
    hdrs = [
        "android_url_loader.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":android_resource_loader",
        ":url_loader",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/async:future_interrupter",
        "//core/common",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/monitor:profiling_clock",
        "//core/view/platforms/android/wrappers:activity_context",
        "//core/view/platforms/android/wrappers:input_stream",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

cc_library(
    name = "cronet_url_loader",
    srcs = [
        "cronet_url_loader.cc",
    ],
    hdrs = [
        "cronet_url_loader.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":android_resource_loader",
        ":url_loader",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/async:future_interrupter",
        "//core/common",
        "//core/common:context",
        "//core/common:jni_helpers",
        "//core/monitor:profiling_clock",
        "//core/view/platforms/android/wrappers:activity_context",
        "//core/view/platforms/android/wrappers:input_stream",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
    ] + if_non_android(["@bazel_tools//tools/jdk:jni"]),
    alwayslink = 1,
)

# blaze test :android_url_loader_robo_test --copt="-DROBOLECTRIC_IMP_PLATFORM=1"
# ((broken link) forces us to explicitly specify the copt in order to expose
# supported Robolectric android types from activity_context.h.)

# blaze test :cronet_url_loader_robo_test --copt="-DROBOLECTRIC_IMP_PLATFORM=1"

cc_library(
    name = "curl_url_loader",
    srcs = [
        "curl_url_loader.cc",
    ],
    hdrs = [
        "curl_url_loader.h",
    ],
    copts = imp_google3_copts(),
    linkopts = ["-lcurl"],
    deps = [
        ":url_loader",
        "//core:config",
        "//core/async:executor",
        "//core/async:future",
        "//core/common",
        "//core/common:robin_map",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "emscripten_url_loader",
    srcs = [
        "emscripten_url_loader.cc",
    ],
    hdrs = [
        "emscripten_url_loader.h",
    ],
    copts = imp_google3_copts(),
    tags = ["nobuilder"],
    deps = [
        ":url_loader",
        "//core/async:future",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
    ],
    alwayslink = 1,
)

objc_library(
    name = "ios_url_loader",
    srcs = [
        "ios_url_loader.mm",
    ],
    hdrs = [
        "ios_url_loader.h",
    ],
    features = ["-layering_check"],
    sdk_dylibs = [
        "libc++",
    ],
    deps = [
        ":url_loader",
        "//core/common",
        "//core/monitor:profiling_clock",
        "//third_party/apple_frameworks:Foundation",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/types:span",
    ],
)

objc_library(
    name = "ios_url_loader_test_helper",
    srcs = ["ios_url_loader_test_helper.mm"],
    hdrs = ["ios_url_loader_test_helper.h"],
    features = ["-layering_check"],
    sdk_dylibs = ["libc++"],
    deps = [
        "//third_party/apple_frameworks:Foundation",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "resources",
    srcs = [
        "resource_definition.cc",
        "resource_manager.cc",
    ],
    hdrs = [
        "resource_definition.h",
        "resource_manager.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":url_loader",
        "//core:config",
        "//core/async:background_delete",
        "//core/async:executor",
        "//core/async:future",
        "//core/common",
        "//core/common:context",
        "//core/common:robin_map",
        "//core/common:trace",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/base:no_destructor",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@com_google_sandboxed_api//sandboxed_api:embed_file",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
    ] + select({
        "//core:ios": [":ios_url_loader"],
        "//core:android": [":android_url_loader"],
        "//core:emscripten": [":emscripten_url_loader"],
        "//conditions:default": [":curl_url_loader"],
    }) + if_dev_runtime(["@com_google_absl//absl/container:btree"]),
    alwayslink = 1,
)

imp_resources(
    name = "test_resources",
    srcs = [
        ":test_data/test.txt",
        ":test_data/test_model.glb",
    ],
    namespace = "imp::test_data",
    out_header = "test_resources.h",
)

imp_resources(
    name = "implicit_test_resources",
    srcs = [
        ":test_data/implicit.txt",
    ],
    namespace = "imp::test_data",
    out_header = "implicit_test_resources.h",
)

imp_resources(
    name = "remote_test_resources",
    srcs = [
        ":test_data/remote.txt",
    ],
    base_url = "(broken link)/",
    embed = False,
    namespace = "imp::test_data",
    out_header = "remote_test_resources.h",
    urls = {
        ":test_data/remote.txt": "path/to/remote.txt",
    },
)

# Creating a cc_library just to wrap test_resources, as the obj_library
# build command does not take non-cc_libraries as dependencies.
cc_library(
    name = "remote_test_resources_cc",
    deps = [
        ":remote_test_resources",
    ],
    alwayslink = 1,
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

# Creating a cc_library just to wrap test_resources, as the obj_library
# build command does not take non-cc_libraries as dependencies.
cc_library(
    name = "implicit_test_resources_cc",
    deps = [
        ":implicit_test_resources",
    ],
    alwayslink = 1,
)

bzl_library(
    name = "portable_embed_data_bzl",
    srcs = ["portable_embed_data.bzl"],
    deps = ["@rules_cc//cc:core_rules"],
    # TODO: restore visibility restriction.
    # visibility = ["//visibility:private"],
)

bzl_library(
    name = "resources_bzl",
    srcs = ["resources.bzl"],
    # TODO: restore visibility restriction.
    # visibility = ["//visibility:private"],
    deps = [
        "//build_tools:imp_bzl",
        "//build_tools:path_tools_bzl",
        "@bazel_skylib//lib:paths",
        "@rules_cc//cc:core_rules",
    ],
)

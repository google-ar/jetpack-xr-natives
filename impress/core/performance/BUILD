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
load(
    "//build_tools:imp.bzl",
    "imp_google3_copts",
    "imp_malloc_wrap_linkopts",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "profiler",
    srcs = [
        "profiler.cc",
        "timer.cc",
    ],
    hdrs = [
        "profiler.h",
        "profiler_state.h",
        "profiler_structs.h",
        "timer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":memory_stats",
        "//core:config",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "memory_stats",
    srcs = select({
        "@platforms//os:osx": [
            "darwin/editor_malloc.cc",
            "darwin/memory_stats.cc",
        ],
        "@platforms//os:ios": [
            "darwin/editor_malloc.cc",
            "darwin/memory_stats.cc",
        ],
        ":android_and_memory_graph": [
            "android/editor_malloc.cc",
            "android/memory_stats.cc",
        ],
        "@platforms//os:android": [
            "default/memory_stats.cc",
        ],
        "@platforms//os:emscripten": [
            "wasm/editor_malloc.cc",
            "wasm/memory_stats.cc",
        ],
        "//conditions:default": [
            "default/memory_stats.cc",
        ],
    }),
    hdrs = select({
        ":android_and_memory_graph": [
            "android/editor_malloc.h",
            "memory_stats.h",
        ],
        "//conditions:default": [
            "memory_stats.h",
        ],
    }),
    linkopts = select({
        ":android_and_memory_graph": imp_malloc_wrap_linkopts(),
        "@platforms//os:android": [],
        "@platforms//os:emscripten": imp_malloc_wrap_linkopts(),
        "//conditions:default": [],
    }),
    deps = [
        "//core:config",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/debugging:stacktrace",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/synchronization",
    ],
    alwayslink = 1,
)

config_setting(
    name = "imp_profiler_enable_memory_graph",
    define_values = {
        "IMP_PROFILER_MEMORY_GRAPH": "1",
    },
)

# TODO: This is a temporary fix, as imp_malloc_wrap_linkopts causes a crash in maps.
config_setting(
    name = "android_and_memory_graph",
    constraint_values = ["@platforms//os:android"],
    define_values = {"IMP_PROFILER_MEMORY_GRAPH": "1"},
)

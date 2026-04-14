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
load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "performance",
    srcs = [
        "callstack_panel.cc",
        "flame_graph.cc",
        "frame_time_panel.cc",
        "hierarchy_panel.cc",
        "imgui_helper.cc",
        "memory_panel.cc",
        "performance_window.cc",
        "render_info_panel.cc",
    ],
    hdrs = [
        "callstack_panel.h",
        "flame_graph.h",
        "flame_graph_colors.h",
        "frame_time_panel.h",
        "hierarchy_panel.h",
        "imgui_helper.h",
        "memory_panel.h",
        "performance_window.h",
        "render_info_panel.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":circular_buffer",
        ":panel",
        ":sample_processor",
        "//core:config",
        "//core/common:rememberer",
        "//core/common:trace",
        "//core/editor:widget",
        "//core/ncsb",
        "//core/performance:memory_stats",
        "//core/performance:profiler",
        "//core/sprite",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/debugging:leak_check",
        "@com_google_absl//absl/debugging:symbolize",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@dear_imgui",
        "@implot",
    ],
    alwayslink = 1,
)

cc_library(
    name = "panel",
    hdrs = [
        "config.h",
        "monitor_panel.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/time",
    ],
    alwayslink = 1,
)

cc_library(
    name = "circular_buffer",
    hdrs = [
        "circular_buffer.h",
    ],
    copts = imp_google3_copts(),
    alwayslink = 1,
)

cc_library(
    name = "sample_processor",
    srcs = ["sample_processor.cc"],
    hdrs = [
        "sample_processor.h",
        "sample_processor_types.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:trace",
        "//core/performance:profiler",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

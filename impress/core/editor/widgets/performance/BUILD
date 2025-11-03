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
        "performance_window.cc",
    ],
    hdrs = [
        "performance_window.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":default_panels",
        ":panel",
        "//core:config",
        "//core/common:rememberer",
        "//core/common:trace",
        "//core/editor:widget",
        "//core/ncsb",
        "//core/performance:profiler",
        "//core/view:base_view",
        "//core/view:view_events",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/debugging:leak_check",
        "@com_google_absl//absl/strings",
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
    name = "default_panels",
    srcs = [
        "frame_time_panel.cc",
        "hierarchy_panel.cc",
        "render_info_panel.cc",
    ],
    hdrs = [
        "frame_time_panel.h",
        "hierarchy_panel.h",
        "render_info_panel.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":circular_buffer",
        ":panel",
        "//core/common:trace",
        "//core/ncsb",
        "//core/performance:profiler",
        "//core/sprite",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:view_config_proto_cc_imp",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
        "@implot",
    ],
    alwayslink = 1,
)

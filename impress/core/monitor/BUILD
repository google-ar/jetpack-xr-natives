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

# Monitor implements Performance Monitoring for the Impress library
# See (broken link)

load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "monitor",
    srcs = [
        "duration_measurement.cc",
        "duration_measurement_data.cc",
        "impress_log_helpers.cc",
        "monitor.cc",
        "scoped_duration_measurement.cc",
        "value_measurement.cc",
        "value_measurement_data.cc",
    ],
    hdrs = [
        "duration_measurement.h",
        "duration_measurement_data.h",
        "duration_report_writer.h",
        "impress_log_helpers.h",
        "measurement_data.h",
        "monitor.h",
        "monitor_helpers.h",
        "scoped_duration_measurement.h",
        "value_measurement.h",
        "value_measurement_data.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":profiling_clock",
        ":simple_histogram",
        "//core/common:log",
        "//core/common:platform_helpers",
        "//core/common:typed_ids",
        "//core/math:moving_average",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:variant",
        "@mediapipe//mediapipe/framework/deps:clock",
        "@mediapipe//mediapipe/framework/port:status",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "profiling_clock",
    srcs = [
        "profiling_clock.cc",
    ],
    hdrs = [
        "profiling_clock.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework/deps:clock",
    ],
    alwayslink = 1,
)

cc_library(
    name = "simple_histogram",
    srcs = ["simple_histogram.cc"],
    hdrs = ["simple_histogram.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "@com_google_absl//absl/types:span",
    ],
    alwayslink = 1,
)

cc_library(
    name = "monitor_summary",
    srcs = [
        "monitor_summary.cc",
    ],
    hdrs = [
        "monitor_summary.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":monitor",
        ":simple_histogram",
        ":windowed_average",
        "//core/math",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/types:variant",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "windowed_average",
    hdrs = [
        "windowed_average.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        "//core/math",
        "//core/math:moving_average",
        "@com_google_absl//absl/strings:str_format",
    ],
    alwayslink = 1,
)

cc_library(
    name = "performance_report_writer",
    srcs = [
        "performance_report_writer.cc",
    ],
    hdrs = [
        "performance_report_writer.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = 1,
    deps = [
        ":monitor",
        "//logs/proto/vr/arcore/impress:impress_log_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@mediapipe//mediapipe/framework/deps:clock",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "default_monitor_summary",
    srcs = [
        "default_monitor_summary.cc",
    ],
    hdrs = [
        "default_monitor_summary.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":monitor",
        ":monitor_summary",
        "//core/common:log",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/time",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "frame_loop_watcher",
    srcs = [
        "frame_loop_watcher.cc",
    ],
    hdrs = [
        "frame_loop_watcher.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":monitor",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view:view_events",
    ],
    alwayslink = 1,
)

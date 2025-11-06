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
load("//build_tools:imp.bzl", "if_android", "if_emscripten", "imp_copts", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "task_scheduler",
    srcs = ["task_scheduler.cc"],
    hdrs = ["task_scheduler.h"],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":task",
        ":task_priority",
        "//core/common:invocable",
        "//core/common:robin_map",
        "@com_google_absl//absl/base:nullability",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "task_priority",
    hdrs = ["task_priority.h"],
    copts = imp_copts(),
    linkstatic = True,
    alwayslink = 1,
)

cc_library(
    name = "executor",
    srcs = ["executor.cc"],
    hdrs = ["executor.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":task",
        ":task_priority",
        "//core/common:invocable",
        "//core/common:platform_helpers",
        "//core/common:string_helpers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "executor_helpers",
    srcs = ["executor_helpers.cc"],
    hdrs = ["executor_helpers.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":executor",
        ":future",
        ":simple_executor",
        ":thread_pool_executor",
        "//core:config",
        "//core/common:invocable",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:optional",
    ] + if_emscripten(
        [],
        [
        ],
    ),
    alwayslink = 1,
)

cc_library(
    name = "future",
    srcs = [
        "future_impl.cc",
    ],
    hdrs = [
        "future.h",
        "future_common.h",
        "future_impl.h",
        "future_traits.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":executor",
        ":task",
        ":task_priority",
        "//core/common",
        "//core/common:holdable",
        "//core/common:invocable",
        "//core/common:type_erased",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:variant",
        "@com_google_glog//:glog",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "simple_executor",
    srcs = ["simple_executor.cc"],
    hdrs = ["simple_executor.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":executor",
        ":task",
        ":task_priority",
        ":task_scheduler",
        "//core/common:invocable",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
    alwayslink = 1,
)

cc_library(
    name = "thread_pool_executor",
    srcs = ["thread_pool_executor.cc"],
    hdrs = ["thread_pool_executor.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":executor",
        ":task",
        ":task_scheduler",
        "//core:config",
        "//core/common",
        "//core/common:invocable",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/libs:utils",
    ] + if_android([
    ]) + if_emscripten(
        [],
        [],
    ),
    alwayslink = 1,
)

cc_library(
    name = "future_interrupter",
    srcs = ["future_interrupter.cc"],
    hdrs = ["future_interrupter.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":future",
        "@com_google_absl//absl/synchronization",
    ],
    alwayslink = 1,
)

cc_library(
    name = "future_status_utils",
    srcs = ["future_status_utils.cc"],
    hdrs = ["future_status_utils.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":future",
        "@com_google_absl//absl/status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "future_container_utils",
    hdrs = ["future_container_utils.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":future",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/status",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "task",
    srcs = [
        "task.cc",
    ],
    hdrs = [
        "task.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        ":task_priority",
        "//core/common:invocable",
        "@com_google_absl//absl/time",
    ],
    alwayslink = 1,
)

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
    "//build_tools:imp.bzl",
    "imp_google3_copts",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "input_action_event",
    hdrs = [
        "input_action_event.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:log",
        "//core/common:platform_helpers",
        "//core/math",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "controller_input_handler",
    srcs = ["controller_input_handler.cc"],
    hdrs = [
        "controller_input_handler.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":controller_events",
        ":input_action_event",
        "//:api",
        "//core/collision:ray",
        "//core/common:platform_helpers",
        "//core/input",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "controller_events",
    srcs = ["controller_events.cc"],
    hdrs = [
        "controller_events.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":action_config",
        ":input_action_event",
        "//core/collision:ray",
        "//core/common:log",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/types:optional",
    ],
    alwayslink = 1,
)

cc_library(
    name = "action_config",
    hdrs = [
        "action_config.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

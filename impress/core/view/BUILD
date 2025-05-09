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
    "if_dev_runtime",
    "imp_defines",
    "imp_google3_copts",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

# Target defining the interface of imp::View.
cc_library(
    name = "base_view",
    srcs = ["base_view.cc"],
    hdrs = ["base_view.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":view_hooks",
        "//core:config",
        "//core/common:enum_flags",
        "//core/common:pass_key",
        "//core/common:rememberer",
        "//core/math",
        "//core/ncsb:node_flag",
        "//core/render:base_renderable_manager",
        "//core/split_engine:split_engine_serializer",
        "//core/view/scripting",
        "//core/view/utils:frame_time",
        "//core/view/utils:view_config_proto_cc_imp",
        "//core/window:filament_host",
        "//core/window:window_rotation",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ],
    alwayslink = 1,
)

cc_library(
    name = "view_events",
    srcs = ["view_events.cc"],
    hdrs = ["view_events.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//core/math",
        "//core/ncsb",
        "//core/window:window_rotation",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:cord",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "view_hooks",
    hdrs = ["view_hooks.h"],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//core/common:invocable",
        "//core/math",
        "//core/window:filament_host",
        "//core/window:window_rotation",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "view_host",
    srcs = [
        "view_host.cc",
        "view_state.cc",
    ],
    hdrs = [
        "view_host.h",
        "view_state.h",
    ],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    linkstatic = True,
    deps = [
        ":base_view",
        ":view_events",
        ":view_hooks",
        "//core:config",
        "//core/async:executor",
        "//core/async:thread_pool_executor",
        "//core/common",
        "//core/common:filament_engine_helpers",
        "//core/common:pass_key",
        "//core/common:registry",
        "//core/common:trace",
        "//core/math",
        "//core/ncsb",
        "//core/window:filament_host",
        "//core/window:window_rotation",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
    ] + if_dev_runtime([
        "//core/editor:editor_impl",
        "//core/common:invocable",
        "@dear_imgui",
        "//core/editor",
        "//core/editor:editor_plugin",
        "//core/window:dev_mode_extension",
        "//core/input:dev_mode_input_interceptor",
    ]),
    alwayslink = 1,
)

cc_library(
    name = "app_sandbox_target",
    srcs = [
        "app_sandbox_target.cc",
    ],
    linkstatic = True,
    deps = ["//core/view/framework"],
    alwayslink = 1,
)

licenses(["notice"])

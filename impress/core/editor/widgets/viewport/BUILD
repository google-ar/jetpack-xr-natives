# Copyright 2026 Google LLC
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
load("//build_tools:imp.bzl", "imp_defines", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "viewport_render_target",
    srcs = ["viewport_render_target.cc"],
    hdrs = ["viewport_render_target.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        "//core/math",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "viewport_widget",
    srcs = ["viewport_widget.cc"],
    hdrs = ["viewport_widget.h"],
    copts = imp_google3_copts(),
    defines = imp_defines(),
    deps = [
        ":viewport_helpers",
        ":viewport_render_target",
        "//core:config",
        "//core/editor",
        "//core/editor:editor_info",
        "//core/editor:file_loader_helper",
        "//core/editor:widget",
        "//core/editor/ui:drag_and_drop",
        "//core/geometry/shapes",
        "//core/math",
        "//core/view:base_view",
        "//core/window:filament_host",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/strings:string_view",
        "@dear_imgui",
    ],
    alwayslink = 1,
)

cc_library(
    name = "viewport_helpers",
    hdrs = ["viewport_helpers.h"],
    deps = [
        "//core/math",
        "//core/ncsb",
        "//core/view/utils:device",
    ],
    alwayslink = 1,
)

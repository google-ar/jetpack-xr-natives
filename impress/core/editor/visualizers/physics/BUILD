# Copyright 2025 Google LLC
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

# IMGUI UI Widgets for Impress Editor.
load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "visualize_physics_collidables",
    srcs = [
        "visualize_physics_collidables.cc",
    ],
    hdrs = [
        "visualize_physics_collidables.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/common:debug_draw",
        "//core/common:rememberer",
        "//core/editor",
        "//core/editor:events",
        "//core/editor:widget",
        "//core/ncsb",
        "//core/physics",
        "//core/view:base_view",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

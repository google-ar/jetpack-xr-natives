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
    name = "glyph_atlas_visualizer",
    srcs = [
        "glyph_atlas_visualizer.cc",
    ],
    hdrs = [
        "glyph_atlas_visualizer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/common:invocable",
        "//core/common:rememberer",
        "//core/editor:widget",
        "//core/editor/layout",
        "//core/geometry/shapes",
        "//core/math",
        "//core/render",
        "//core/view:base_view",
        "@com_google_absl//absl/debugging:leak_check",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
        "@implot",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

cc_library(
    name = "sliced_glyph_atlas_visualizer",
    srcs = [
        "sliced_glyph_atlas_visualizer.cc",
    ],
    hdrs = [
        "sliced_glyph_atlas_visualizer.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core:config",
        "//core/common:invocable",
        "//core/common:rememberer",
        "//core/editor:widget",
        "//core/editor/layout",
        "//core/geometry/shapes",
        "//core/math",
        "//core/render",
        "//core/view:base_view",
        "@com_google_absl//absl/debugging:leak_check",
        "@com_google_absl//absl/strings",
        "@dear_imgui",
        "@implot",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:pixel_buffer_descriptor",
    ],
    alwayslink = 1,
)

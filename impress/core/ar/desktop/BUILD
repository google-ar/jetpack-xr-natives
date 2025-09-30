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

load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_assets(
    name = "ar_desktop_data",
    srcs = [
        "//third_party/arcore/googledata/sceneform/assets/ar_desktop:apartment.glb",
    ],
    namespace = "ar_desktop_data",
)

cc_library(
    name = "desktop",
    srcs = [
        "ar_session_native_desktop.cc",
        "desktop_camera_controller.cc",
    ],
    hdrs = [
        "ar_session_native_desktop.h",
        "desktop_camera_controller.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":ar_desktop_data",
        "//:api",
        "//core/ar:ar_hdr_lighting",
        "//core/ar:shared",
        "//core/collision:collision_helpers",
        "//core/common",
        "//core/common:debug_draw",
        "//core/common:robin_map",
        "//core/input",
        "//core/lighting:environment_light_factory",
        "//core/math",
        "//core/view/framework",
        "@com_google_absl//absl/memory",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@dear_imgui",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//gl",
    ],
    alwayslink = 1,
)

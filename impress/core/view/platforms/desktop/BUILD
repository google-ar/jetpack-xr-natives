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

load("//build_tools:imp.bzl", "imp_google3_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

# An incomplete library which is combined via .bzl with a provider of the client API.
cc_library(
    name = "desktop",
    srcs = [
        "desktop_main.cc",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:log",
        "//core/view:view_host",
        "//core/view/framework",
        "//core/window:sdl_venue",
        "@com_google_absl//absl/debugging:failure_signal_handler",
        "@com_google_absl//absl/debugging:symbolize",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@mediapipe//mediapipe/framework/port:status",
        "@third_party//filament",
    ],
    alwayslink = 1,
)

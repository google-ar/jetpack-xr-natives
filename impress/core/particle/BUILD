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
load("//build_tools:imp.bzl", "imp_copts")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

cc_library(
    name = "particle",
    srcs = [
        "data_layout.cc",
        "particle_data.cc",
        "particle_emitter_info.cc",
        "standard_particle_data_provider.cc",
        "standard_particle_service.cc",
    ],
    hdrs = [
        "data_layout.h",
        "particle_data.h",
        "particle_data_provider.h",
        "particle_emitter_info.h",
        "particle_service.h",
        "standard_particle_data_provider.h",
        "standard_particle_service.h",
    ],
    copts = imp_copts(),
    linkstatic = True,
    deps = [
        "//core/async:future",
        "//core/common:owned_ptr",
        "//core/math",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/memory",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

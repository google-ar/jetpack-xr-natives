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

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "interaction_states",
    hdrs = [
        "interaction_owner.h",
        "interaction_states.h",
    ],
    deps = [
        "//core/collision:ray",
        "//core/common:smooth",
        "//core/math",
        "//core/ncsb",
        "//extensions/sceneviewerxr/ux:footprint",
        "//extensions/sceneviewerxr/ux:interaction_mode",
        "//extensions/sceneviewerxr/ux:ramp",
        "//extensions/sceneviewerxr/ux:state_machine",
        "@com_google_absl//absl/time",
    ],
    alwayslink = 1,
)

cc_library(
    name = "idle",
    srcs = ["idle.cc"],
    hdrs = ["idle.h"],
    deps = [
        ":interaction_states",
        "//core/collision:ray",
        "//core/common:enum_flags",
        "//core/common:smooth",
        "//core/math",
        "//core/ncsb",
        "//core/view/utils:frame_time",
        "//extensions/sceneviewerxr/ux:constants",
        "//extensions/sceneviewerxr/ux:footprint",
        "//extensions/sceneviewerxr/ux:input_flag",
        "//extensions/sceneviewerxr/ux:interaction_mode",
        "//extensions/sceneviewerxr/ux:ramp",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

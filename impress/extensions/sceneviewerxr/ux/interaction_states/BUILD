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
    srcs = ["interaction_owner.cc"],
    hdrs = [
        "interaction_owner.h",
        "interaction_states.h",
    ],
    deps = [
        "//core/camera",
        "//core/collision:ray",
        "//core/common:smooth",
        "//core/math",
        "//core/ncsb",
        "//extensions/sceneviewerxr/ux:constants",
        "//extensions/sceneviewerxr/ux:footprint",
        "//extensions/sceneviewerxr/ux:interaction_mode",
        "//extensions/sceneviewerxr/ux:ramp",
        "//extensions/sceneviewerxr/ux:state_machine",
        "//extensions/sceneviewerxr/ux:ui_event_listener",
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

cc_library(
    name = "initial",
    srcs = ["initial.cc"],
    hdrs = ["initial.h"],
    deps = [
        ":interaction_states",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//extensions/sceneviewerxr/ux:constants",
        "//extensions/sceneviewerxr/ux:gltf_bounds",
    ],
    alwayslink = 1,
)

cc_library(
    name = "one_handed_scale",
    srcs = ["one_handed_scale.cc"],
    hdrs = ["one_handed_scale.h"],
    deps = [
        ":idle",
        ":interaction_states",
        "//core/camera",
        "//core/collision:collision_helpers",
        "//core/collision:ray",
        "//core/common:enum_flags",
        "//core/math",
        "//core/ncsb",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "//extensions/sceneviewerxr/ux:gltf_bounds",
        "//extensions/sceneviewerxr/ux:input_flag",
        "//extensions/sceneviewerxr/ux:interaction_mode",
        "//extensions/sceneviewerxr/ux:state_machine",
    ],
    alwayslink = 1,
)

cc_library(
    name = "opening",
    srcs = ["opening.cc"],
    hdrs = ["opening.h"],
    deps = [
        ":idle",
        ":interaction_states",
        "//core/view/utils:frame_time",
        "//extensions/sceneviewerxr/ux:constants",
    ],
    alwayslink = 1,
)

cc_library(
    name = "rotation",
    srcs = ["rotation.cc"],
    hdrs = ["rotation.h"],
    deps = [
        ":idle",
        ":interaction_states",
        "//core/collision:ray",
        "//core/common:enum_flags",
        "//core/math",
        "//core/ncsb",
        "//core/view/utils:frame_time",
        "//extensions/sceneviewerxr/ux:footprint",
        "//extensions/sceneviewerxr/ux:input_flag",
        "//extensions/sceneviewerxr/ux:interaction_mode",
    ],
    alwayslink = 1,
)

cc_library(
    name = "scale_reset",
    srcs = ["scale_reset.cc"],
    hdrs = ["scale_reset.h"],
    deps = [
        ":idle",
        ":interaction_states",
        "//core/common:smooth",
        "//core/view/utils:frame_time",
        "//extensions/sceneviewerxr/ux:constants",
        "//extensions/sceneviewerxr/ux:interaction_mode",
        "//extensions/sceneviewerxr/ux:state_machine",
    ],
    alwayslink = 1,
)

cc_library(
    name = "shutdown",
    srcs = ["shutdown.cc"],
    hdrs = ["shutdown.h"],
    deps = [
        ":interaction_states",
        "//core/common:smooth",
        "//core/view/utils:frame_time",
        "//extensions/sceneviewerxr/ux:constants",
        "//extensions/sceneviewerxr/ux:state_machine",
    ],
    alwayslink = 1,
)

cc_library(
    name = "translation",
    srcs = ["translation.cc"],
    hdrs = ["translation.h"],
    deps = [
        ":idle",
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
        "@com_google_absl//absl/time",
    ],
    alwayslink = 1,
)

cc_library(
    name = "two_handed_scale",
    srcs = ["two_handed_scale.cc"],
    hdrs = ["two_handed_scale.h"],
    deps = [
        ":interaction_states",
        "//core/collision:ray",
        "//core/common:enum_flags",
        "//core/common:smooth",
        "//core/math",
        "//core/ncsb",
        "//core/view/utils:frame_time",
        "//extensions/sceneviewerxr/ux:constants",
        "//extensions/sceneviewerxr/ux:input_flag",
        "//extensions/sceneviewerxr/ux:interaction_mode",
        "//extensions/sceneviewerxr/ux:state_machine",
    ],
    alwayslink = 1,
)

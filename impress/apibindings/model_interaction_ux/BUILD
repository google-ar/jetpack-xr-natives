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
load(
    "//build_tools:imp.bzl",
    "imp_google3_copts",
)

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reform_affordance_ux",
    srcs = [
        "scene_viewer_component.cc",
    ],
    hdrs = [
        "scene_viewer_component.h",
    ],
    copts = imp_google3_copts(),
    features = ["-layering_check"],
    deps = [
        "//:api",
        "//core/audio",
        "//core/camera",
        "//core/collision:collision_helpers",
        "//core/collision:ray",
        "//core/common:debug_draw",
        "//core/common:enum_flags",
        "//core/common:smooth",
        "//core/geometry/shapes",
        "//core/math",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "//extensions/sceneviewerxr/ux:constants",
        "//extensions/sceneviewerxr/ux:footprint",
        "//extensions/sceneviewerxr/ux:input_flag",
        "//extensions/sceneviewerxr/ux:interaction_mode",
        "//extensions/sceneviewerxr/ux:ui_event_listener",
        "//extensions/sceneviewerxr/ux/interaction_states",
        "//extensions/sceneviewerxr/ux/interaction_states:idle",
        "//extensions/sceneviewerxr/ux/interaction_states:initial",
        "//extensions/sceneviewerxr/ux/interaction_states:rotation",
        "//extensions/sceneviewerxr/ux/interaction_states:scale_reset",
        "//extensions/sceneviewerxr/ux/interaction_states:translation",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_impress//core/common:log",
        "@third_party//split_engine/input",
        "@third_party//split_engine/input:split_engine_input_event",
    ],
    alwayslink = 1,
)

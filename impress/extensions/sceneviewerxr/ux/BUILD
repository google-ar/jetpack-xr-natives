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

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

cc_library(
    name = "ramp",
    hdrs = ["ramp.h"],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:enum_flags",
        "//core/common:platform_helpers",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "state_machine",
    hdrs = ["state_machine.h"],
    copts = imp_google3_copts(),
    deps = [
        "//core/common:enum_flags",
        "//core/common:platform_helpers",
        "//core/common:visit",
        "//core/view/utils:frame_time",
        "@com_google_absl//absl/time",
        "@com_google_impress//core/common:log",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "interaction_mode",
    hdrs = ["interaction_mode.h"],
    alwayslink = 1,
)

cc_library(
    name = "gltf_bounds",
    srcs = ["gltf_bounds.cc"],
    hdrs = ["gltf_bounds.h"],
    deps = [
        "//core/common:filament_helpers",
        "//core/common:platform_helpers",
        "//core/common:typed_ids",
        "//core/geometry/shapes",
        "//core/math",
        "//core/model",
        "//core/model:model_data_details",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:utils_hdrs",
    ],
    alwayslink = 1,
)

cc_library(
    name = "footprint",
    srcs = ["footprint.cc"],
    hdrs = ["footprint.h"],
    copts = imp_google3_copts(),
    deps = [
        ":gltf_bounds",
        ":interaction_mode",
        ":ramp",
        ":state_machine",
        "//:api",
        "//core/assets",
        "//core/async:future",
        "//core/geometry/shapes",
        "//core/math",
        "//core/model",
        "//core/model:model_data_details",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/utils:frame_time",
        "//extensions/sceneviewerxr/assets:footprint_assets",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:optional",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//filament/libs:utils_hdrs",
        "@third_party//split_engine/materials:svxr_footprint_material",
    ],
    alwayslink = 1,
)

cc_library(
    name = "input_flag",
    srcs = ["input_flag.cc"],
    hdrs = ["input_flag.h"],
    deps = [
        "//core/common:enum_flags",
        "@third_party//split_engine/input:split_engine_input_event",
    ],
    alwayslink = 1,
)

cc_library(
    name = "constants",
    hdrs = ["constants.h"],
    deps = [
        "//core/common:smooth",
        "@com_google_absl//absl/time",
    ],
    alwayslink = 1,
)

cc_library(
    name = "sceneviewerxr_ux",
    srcs = [
        "a11y_control.cc",
        "load_indicator_component.cc",
        "menu_panel.cc",
        "plane.cc",
        "scale_indicator.cc",
        "scene_viewer_component.cc",
    ],
    hdrs = [
        "a11y_control.h",
        "load_indicator_component.h",
        "menu_panel.h",
        "plane.h",
        "scale_indicator.h",
        "scene_viewer_component.h",
    ],
    copts = imp_google3_copts(),
    features = ["-layering_check"],
    deps = [
        ":constants",
        ":footprint",
        ":gltf_bounds",
        ":input_flag",
        ":interaction_mode",
        ":ramp",
        ":state_machine",
        "//:api",
        "//core/assets",
        "//core/async:future",
        "//core/audio",
        "//core/collision:collision_helpers",
        "//core/collision:ray",
        "//core/common:debug_draw",
        "//core/common:enum_flags",
        "//core/common:smooth",
        "//core/geometry/shapes",
        "//core/math",
        "//core/model/mesh:mesh_data",
        "//core/ncsb",
        "//core/render",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:frame_time",
        "//extensions/sceneviewerxr/ux/interaction_states",
        "//extensions/sceneviewerxr/ux/interaction_states:idle",
        "//vr/android_xr/sceneviewerxr/api:sceneviewerxr_session_listener",
        "//vr/android_xr/sceneviewerxr/assets:sceneviewerxr_assets",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:string_view",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
        "@com_google_impress//core/common:log",
        "@third_party//OpenXR_KHR:generated_openxr_headers",
        "@third_party//filament/filament:filament_hdrs",
        "@third_party//filament/filament/backend:driver_enums",
        "@third_party//filament/libs:math_hdrs",
        "@third_party//split_engine:subspace_root",
        "@third_party//split_engine/input",
        "@third_party//split_engine/input:split_engine_input_event",
        "@third_party//split_engine/materials:svxr_plane_material",
    ],
    alwayslink = 1,
)

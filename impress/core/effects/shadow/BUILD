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
load("//build_tools:assets.bzl", "imp_assets")
load("//build_tools:imp.bzl", "imp_google3_copts")
load("//build_tools:imp_material.bzl", "imp_material")
load(
    "//core/proto:imp_cc_proto_library.bzl",
    "imp_cc_proto_library",
)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_material(
    name = "simple_white",
    src = "simple_white.mat",
)

imp_material(
    name = "single_axis_blur",
    src = "single_axis_blur.mat",
    includes = ["gaussian_kernel.glsl"],
)

imp_assets(
    name = "contact_shadow_assets",
    srcs = [
        ":simple_white",
        ":single_axis_blur",
    ],
    namespace = "imp::contact_shadow_assets",
)

proto_library(
    name = "contact_shadow_projector_state",
    srcs = [
        "contact_shadow_projector_state.proto",
    ],
    deps = [
        "//core/view/framework:framework_proto",
    ],
)

proto_library(
    name = "contact_shadow_receiver_state",
    srcs = [
        "contact_shadow_receiver_state.proto",
    ],
)

imp_cc_proto_library(
    name = "contact_shadow_projector_state_cc_proto",
    proto = ":contact_shadow_projector_state",
)

imp_cc_proto_library(
    name = "contact_shadow_receiver_state_cc_proto",
    proto = ":contact_shadow_receiver_state",
)

cc_library(
    name = "contact_shadows",
    srcs = [
        "contact_shadow_projector.cc",
        "contact_shadow_receiver.cc",
    ],
    hdrs = [
        "contact_shadow_projector.h",
        "contact_shadow_receiver.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        ":contact_shadow_assets",
        ":contact_shadow_projector_state_cc_proto",
        ":contact_shadow_receiver_state_cc_proto",
        "//core/assets",
        "//core/async:future",
        "//core/common:platform_helpers",
        "//core/math",
        "//core/ncsb",
        "//core/render",
        "//core/render_passes:texture_pipeline_renderer",
        "//core/render_passes:texture_pipeline_renderer_proto_cc_imp",
        "//core/scene_handles",
        "//core/view:base_view",
        "//core/view/framework",
        "//core/view/framework:framework_proto_cc",
        "//core/view/utils:frame_time",
        "//core/view/utils:string_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

imp_material(
    name = "valid_material_test",
    src = "valid_material_test.mat",
)

imp_material(
    name = "invalid_material_test",
    src = "invalid_material_test.mat",
)

imp_assets(
    name = "test_assets",
    srcs = [
        ":invalid_material_test",
        ":valid_material_test",
    ],
    namespace = "imp::contact_shadow_test_assets",
)

filegroup(
    name = "contact_shadows_scuba_goldens",
    srcs = glob(["contact_shadows_scuba_goldens/**/*.png"]),
)

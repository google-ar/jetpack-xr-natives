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

# Impel helps internal devs make performant, high quality AR apps.

load("@bazel_skylib//rules:common_settings.bzl", "bool_flag")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load(
    "//build_tools:imp.bzl",
    "imp_copts",
    "imp_defines",
)

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

## Platform configs ##

config_setting(
    name = "android",
    constraint_values = [
        "@platforms//os:android",
    ],
)

config_setting(
    name = "emscripten",
    constraint_values = [
        "@platforms//os:emscripten",
    ],
)

config_setting(
    name = "ios",
    constraint_values = [
        "@platforms//os:ios",
    ],
)

config_setting(
    name = "ios_simulator_linux",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:ios",
    ],
    values = {
        "host_cpu": "k8",
    },
)

config_setting(
    name = "ios_simulator_linux_metal",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:ios",
    ],
    define_values = {
        "FILAMENT_USES_METAL": "1",
    },
    values = {
        "host_cpu": "k8",
    },
)

config_setting(
    name = "ios_simulator_mac",
    constraint_values = [
        "@platforms//cpu:x86_64",
        "@platforms//os:ios",
    ],
    values = {
        "host_cpu": "darwin_x86_64",
    },
)

config_setting(
    name = "linux",
    constraint_values = [
        "@platforms//os:linux",
    ],
)

config_setting(
    name = "macos",
    constraint_values = [
        "@platforms//os:macos",
    ],
)

config_setting(
    name = "windows",
    constraint_values = [
        "@platforms//os:windows",
    ],
)

## XR configs ##

config_setting(
    name = "imp_include_stereo_variant_by_default",
    define_values = {"IMP_INCLUDE_STEREO_VARIANT_BY_DEFAULT": "1"},
)

## Dev configs ##

config_setting(
    name = "native_debug",
    values = {"define": "imp_native_debug=true"},
)

config_setting(
    name = "impel_dev_runtime",
    define_values = {
        "IMPEL_DEV_RUNTIME": "1",
    },
)

config_setting(
    name = "imp_dev_runtime",
    define_values = {
        "IMP_DEV_RUNTIME": "1",
    },
)

config_setting(
    name = "imp_enable_editor_on_startup",
    define_values = {
        "IMP_ENABLE_EDITOR_ON_STARTUP": "1",
    },
)

config_setting(
    name = "remote_desktop",
    values = {"define": "remote_desktop=true"},
)

config_setting(
    name = "imp_enable_face_tracking",
    define_values = {
        "IMP_ENABLE_FACE_TRACKING": "1",
    },
)

config_setting(
    name = "imp_invert_editor_input",
    define_values = {
        "IMP_INVERT_EDITOR_INPUT": "1",
    },
)

config_setting(
    name = "imp_enable_stereo_type_multiview",
    define_values = {
        "IMP_ENABLE_STEREO_TYPE_MULTIVIEW": "1",
    },
)

config_setting(
    name = "imp_android_external_texture_surface_uses_image_reader",
    define_values = {
        "IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER": "1",
    },
)

config_setting(
    name = "imp_disable_future_validation",
    define_values = {
        "IMP_DISABLE_FUTURE_VALIDATION": "1",
    },
)

bool_flag(
    name = "imp_trace_use_perfetto",
    build_setting_default = False,
)

config_setting(
    name = "imp_perfetto_enabled",
    flag_values = {
        ":imp_trace_use_perfetto": "True",
    },
)

## Build targets ##

cc_library(
    name = "config",
    hdrs = ["config.h"],
    copts = imp_copts(),
    defines = imp_defines(),
    linkstatic = 1,
    alwayslink = 1,
)

# TODO: move this to core/proto/BUILD
filegroup(
    name = "proto_fuzz_corpus",
    srcs = glob([
        "proto/fuzz_corpus/**",
    ]),
)

# TODO: move this to core/proto/BUILD
filegroup(
    name = "proto_json_corpus",
    srcs = glob([
        "proto/json_corpus/**",
    ]),
)

# TODO: move this to core/scripting/BUILD
filegroup(
    name = "scripting_fuzz_corpus",
    srcs = glob([
        "scripting/fuzz_corpus/**",
    ]),
)

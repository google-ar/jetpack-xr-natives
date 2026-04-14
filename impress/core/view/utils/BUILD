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

cc_library(
    name = "asset",
    hdrs = ["asset.h"],
    copts = imp_google3_copts(),
    deps = [
        "//core/resources",
    ],
    alwayslink = 1,
)

cc_library(
    name = "device",
    srcs = ["device.cc"],
    hdrs = [
        "device.h",
    ],
    deps = [
        "//core:config",
        "//core/common",
        "//core/math",
        "@com_google_absl//absl/types:optional",
        "@com_google_impress//core/common:log",
    ],
    alwayslink = 1,
)

cc_library(
    name = "frame_time",
    srcs = ["frame_time.cc"],
    hdrs = [
        "frame_time.h",
    ],
    deps = [
        "//core/common:trace",
        "@com_google_absl//absl/time",
    ],
    alwayslink = 1,
)

cc_library(
    name = "macros",
    hdrs = ["macros.h"],
    copts = imp_google3_copts(),
    alwayslink = 1,
)

cc_library(
    name = "string_hasher",
    hdrs = ["string_hasher.h"],
    copts = imp_google3_copts(),
    deps = [
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

cc_library(
    name = "string_map",
    hdrs = ["string_map.h"],
    copts = imp_google3_copts(),
    deps = [
        ":string_hasher",
        "@com_google_absl//absl/strings",
        "@robin_map",
    ],
    alwayslink = 1,
)

cc_library(
    name = "string_set",
    hdrs = ["string_set.h"],
    copts = imp_google3_copts(),
    deps = [
        ":string_hasher",
        "//core/common:robin_map",
        "@com_google_absl//absl/strings",
    ],
    alwayslink = 1,
)

proto_library(
    name = "render_settings_proto",
    srcs = [
        "proto/render_settings.proto",
    ],
    deps = [
        "//core/math:math_proto",
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "render_settings_proto_cc_imp",
    proto = ":render_settings_proto",
    deps = [
        "//core/math:math_proto_cc",
    ],
)

java_lite_proto_library(
    name = "render_settings_java_proto_lite",
    deps = [":render_settings_proto"],
)

cc_library(
    name = "render_setting_utils",
    srcs = ["render_setting_utils.cc"],
    hdrs = ["render_setting_utils.h"],
    copts = imp_google3_copts(),
    deps = [
        ":render_settings_proto_cc_imp",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

proto_library(
    name = "cache_config_proto",
    srcs = [
        "proto/cache_config.proto",
    ],
    deps = [
        "//core/proto:imp_proto",
    ],
)

imp_cc_proto_library(
    name = "cache_config_proto_cc_imp",
    proto = ":cache_config_proto",
    deps = [
        "//core/proto:imp_cc_proto",
    ],
)

cc_library(
    name = "render_state_validator",
    srcs = ["render_state_validator.cc"],
    hdrs = ["render_state_validator.h"],
    copts = imp_google3_copts(),
    deps = [
        ":frame_time",
        "//core/common:platform_helpers",
        "//core/ncsb",
        "//core/view:base_view",
        "//core/view/framework",
        "@com_google_impress//core/common:log",
        "@third_party//filament/filament:filament_hdrs",
    ],
    alwayslink = 1,
)

proto_library(
    name = "view_config_proto",
    srcs = [
        "proto/view_config.proto",
    ],
    deps = [
        ":cache_config_proto",
        ":filament_feature_flag_proto",
        ":render_settings_proto",
    ],
)

imp_cc_proto_library(
    name = "view_config_proto_cc_imp",
    proto = ":view_config_proto",
    deps = [
        ":cache_config_proto_cc_imp",
        ":filament_feature_flag_proto_cc_imp",
        ":render_settings_proto_cc_imp",
        "//core/proto:imp_cc_proto",
    ],
)

proto_library(
    name = "filament_feature_flag_proto",
    srcs = [
        "proto/filament_feature_flag.proto",
    ],
)

java_lite_proto_library(
    name = "filament_feature_flag_java_proto_lite",
    deps = [":filament_feature_flag_proto"],
)

imp_cc_proto_library(
    name = "filament_feature_flag_proto_cc_imp",
    proto = ":filament_feature_flag_proto",
    deps = [
        "//core/proto:imp_cc_proto",
    ],
)

java_lite_proto_library(
    name = "view_config_java_proto_lite",
    deps = [":view_config_proto"],
)

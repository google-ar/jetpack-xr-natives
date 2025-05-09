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
    name = "scene_handle_interface",
    hdrs = [
        "scene_handle_interface.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//core/common:hash",
        "//core/common:type_traits",
        "//core/ncsb",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:variant",
    ],
    alwayslink = 1,
)

cc_library(
    name = "scene_handle_helper",
    srcs = [
        "scene_handle_helper.cc",
    ],
    hdrs = [
        "scene_handle_helper.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":scene_handle_interface",
        "//core/common:log",
        "//core/ncsb",
        "//core/proto:proto_common",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:variant",
    ],
    alwayslink = 1,
)

cc_library(
    name = "scene_handles",
    srcs = [
        "scene_handles.cc",
    ],
    hdrs = [
        "scene_handles.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":scene_handle_helper",
        ":scene_handle_interface",
        "//core/common",
        "//core/common:hash",
        "//core/common:log",
        "//core/common:type_traits",
        "//core/ncsb",
        "//core/proto:proto_common",
        "//core/view/framework",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:variant",
        "@mediapipe//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "scene_handle_status_utils",
    hdrs = [
        "scene_handle_status_utils.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//core/common:type_traits",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
    ],
    alwayslink = 1,
)

# Proto library message for scene handles, which are special types using
# BaseSceneHandle to reference nodes/components within a .isf file.
proto_library(
    name = "scene_handles_proto",
    srcs = [
        "scene_handles.proto",
    ],
    exports = ["//core/proto:imp_proto"],
    deps = [
        "//core/proto:imp_proto",
    ],
)

# Proto library message for scene handles, which are special types using
# BaseSceneHandle to reference nodes/components within a .isf file.
imp_cc_proto_library(
    name = "scene_handles_imp_cc_proto",
    proto = ":scene_handles_proto",
    deps = [":scene_handles"],
)

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

cc_library(
    name = "geometry_helper",
    srcs = ["geometry_helper.cc"],
    hdrs = [
        "geometry_helper.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        "//core/collision:ray",
        "//core/geometry/shapes",
        "//core/math",
    ],
    alwayslink = 1,
)

cc_library(
    name = "geometry",
    srcs = [
        "circumcircle.cc",
        "closest_point.cc",
    ],
    hdrs = [
        "capsule_helper.h",
        "circumcircle.h",
        "closest_point.h",
    ],
    copts = imp_google3_copts(),
    linkstatic = True,
    deps = [
        ":geometry_helper",
        "//core/collision:collision_helpers",
        "//core/collision:ray",
        "//core/geometry/shapes",
        "//core/math",
    ],
    alwayslink = 1,
)

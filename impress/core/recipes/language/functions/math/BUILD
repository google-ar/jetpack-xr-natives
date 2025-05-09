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
    name = "math",
    srcs = [
        "angle.cc",
        "arithmetic.cc",
        "bitwise.cc",
        "constants.cc",
        "exponential.cc",
        "hyperbolic.cc",
        "utility.cc",
        "vector.cc",
    ],
    hdrs = [
        "math.h",
    ],
    copts = imp_google3_copts(),
    deps = [
        "//core/math",
        "//core/recipes/language:base_recipe_system",
        "//core/recipes/language:recipe_graph_cc",
        "//core/recipes/language:recipe_utils",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:string_view",
    ],
    alwayslink = 1,
)

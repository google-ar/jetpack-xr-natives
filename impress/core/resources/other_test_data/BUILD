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

load("//core/resources:resources.bzl", "imp_resources")

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

imp_resources(
    name = "test_resources",
    srcs = [
        "other_test.txt",
    ],
    namespace = "imp::test_data",
    out_header = "test_resources.h",
)

# Creating a cc_library just to wrap test_resources, as the obj_library
# build command does not take non-cc_libraries as dependencies.
cc_library(
    name = "test_resources_cc",
    deps = [
        ":test_resources",
    ],
    alwayslink = 1,
)

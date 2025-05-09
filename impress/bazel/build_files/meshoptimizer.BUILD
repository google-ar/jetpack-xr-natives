# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Fast mesh optimization library with a variety of functionality such as
# vertex cache optimazation, overdraw optimization, compression, etc.
# https://github.com/zeux/meshoptimizer

load("@rules_license//rules:license_kind.bzl", "license_kind")

package(default_visibility = ["//visibility:public"])

license_kind(
    name = "license",
    conditions = [
      "notice",
    ],
)

licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "meshoptimizer",
    srcs = [
        "src/allocator.cpp",
        "src/clusterizer.cpp",
        "src/indexcodec.cpp",
        "src/indexgenerator.cpp",
        "src/overdrawanalyzer.cpp",
        "src/overdrawoptimizer.cpp",
        "src/simplifier.cpp",
        "src/spatialorder.cpp",
        "src/stripifier.cpp",
        "src/vcacheanalyzer.cpp",
        "src/vcacheoptimizer.cpp",
        "src/vertexcodec.cpp",
        "src/vertexfilter.cpp",
        "src/vfetchanalyzer.cpp",
        "src/vfetchoptimizer.cpp",
    ],
    hdrs = ["src/meshoptimizer.h"],
    copts = [
        "-Wno-string-conversion",
        "-Wno-unused-variable",
    ],
    includes = [".", "src"],
    include_prefix = "meshoptimizer",
)

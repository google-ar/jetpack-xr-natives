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

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])

DRACO_DEFINES_FOR_ALL_RELEASED_FEATURES = [
    "DRACO_MESH_COMPRESSION_SUPPORTED",
    "DRACO_POINT_CLOUD_COMPRESSION_SUPPORTED",
    "DRACO_STANDARD_EDGEBREAKER_SUPPORTED",
    "DRACO_PREDICTIVE_EDGEBREAKER_SUPPORTED",
    "DRACO_BACKWARDS_COMPATIBILITY_SUPPORTED",  # Enable all backwards compatibility features.
    "DRACO_NORMAL_ENCODING_SUPPORTED",
    "DRACO_ATTRIBUTE_INDICES_DEDUPLICATION_SUPPORTED",
    "DRACO_ATTRIBUTE_VALUES_DEDUPLICATION_SUPPORTED",
    "DRACO_INTERNAL",
]

draco_root = "src/"
excludes = [
    "src/draco/io/**",
    "src/draco/javascript/**",
    "src/draco/tools/**",
    "src/draco/unity/**",
    "**/*test*"
]

cc_library(
    name = "draco",
    srcs = glob(["**/*.cc"], exclude = excludes),
    hdrs = glob(["**/*.h"], exclude = excludes),
    includes = glob(["**/*.h"], exclude = excludes),
    defines = DRACO_DEFINES_FOR_ALL_RELEASED_FEATURES,
    copts = ["-I" + draco_root],
    deps = [
        "@com_google_absl//absl/base",
    ],
    strip_include_prefix = "src/",
)

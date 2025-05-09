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

# Description:
# A C++ implementation of a fast hash map and hash set using robin hood hashing

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

exports_files(["LICENSE"])

cc_library(
    name = "robin_map",
    hdrs = [
        "include/tsl/robin_growth_policy.h",
        "include/tsl/robin_hash.h",
        "include/tsl/robin_map.h",
        "include/tsl/robin_set.h",
    ],
    copts = ["-fexceptions"],
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    # TODO: why is this necessary? "No file or dir" otherwise...
    include_prefix = "robin_map/",
)


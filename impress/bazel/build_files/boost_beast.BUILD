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
# Boost/Beast (exclusively for varint.hpp)

package(
    default_visibility = ["//visibility:public"],
    features = [
        "-layering_check",
        "-parse_headers",
        "-use_header_modules",  # Incompatible with -std=c++14.
    ],
)

licenses(["notice"])

exports_files(["LICENSE_1_0.txt"])

cc_library(
    name = "boost_beast",
    hdrs = ["include/boost/beast/core/detail/varint.hpp"],
    copts = [
        "-fexceptions",
        "-std=c++17",
    ],
    include_prefix = "boost_beast",
    includes = ["include/boost/beast/core/detail/varint.hpp"],
    strip_include_prefix = "include/boost/beast/core/detail",
)

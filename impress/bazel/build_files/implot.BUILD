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

# ImGui-based plotting library

package(default_visibility = ["//visibility:public"])

licenses([
    "notice",
    "unencumbered",
])

exports_files(["LICENSE.txt"])

cc_library(
    name = "implot",
    srcs = [
        "implot.cpp",
        "implot_items.cpp",
    ],
    hdrs = [
        "implot.h",
        "implot_internal.h",
    ],
    deps = ["@dear_imgui"],
    include_prefix = "implot",
)
